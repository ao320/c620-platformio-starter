#include <Arduino.h>
#include "secrets.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <driver/twai.h>
#include <esp_system.h>
#include "motor_control.h"
#include "position_control.h"
#include "response_metrics.h"
#include "udp_guard.h"
#include "startup_calibration.h"
#include "web_ui.h"

// GPIO21 TX / GPIO22 RX, MCP2562FD VDD=5V, VIO=3.3V, STBY=GND.
// No data TX on boot. Pulse UI: selected IDs 1..3, configured current/1s, then zero/1s.
// Wi-Fi velocity API: targets +/-900 RPM, PI current capped at configured 1000..6000, 250ms lease.
// CAN timing is owned by a separate task, never by HTTP or the browser.
using namespace motor_control;
static const char *AP_SSID = C620_AP_SSID;
static const char *AP_PASSWORD = C620_AP_PASSWORD;
static WebServer server(80);
static String controlToken;
// Passive arm range capture: reject all queued controls and suppress motor data TX.
// Set true to return to passive monitoring (all motor command TX disabled).
static constexpr bool OBSERVATION_ONLY = false;
static uint32_t observationBootId = 0;
static bool apReady = false;
static WiFiUDP udp;
static UdpGuard udpGuard;
static bool udpReady=false,txPowerConfigured=false;
static int8_t txPowerQuarterDbm=0;
static uint32_t udpRejected=0;
static QueueHandle_t commands;
static portMUX_TYPE snapshotMux = portMUX_INITIALIZER_UNLOCKED;
struct Motor {
  bool seen = false;
  uint32_t lastMs = 0, received = 0;
  int16_t rpm = 0, current = 0;
  uint16_t angle = 0;
  uint8_t temp = 0;
  int peakRpm = 0;
  ResponseMetrics response;
  position_control::Encoder encoder;
};
struct Snapshot {
  uint16_t kpMilli[3]={DEFAULT_KP[0],DEFAULT_KP[1],DEFAULT_KP[2]},
      kiMilli[3]={DEFAULT_KI[0],DEFAULT_KI[1],DEFAULT_KI[2]},
      lowKpMilli[3]={DEFAULT_LOW_KP[0],DEFAULT_LOW_KP[1],DEFAULT_LOW_KP[2]};
  uint32_t configAck=0,configRevision=0;
  bool configAccepted=false;
  uint16_t currentLimit=CURRENT;
  startup_calibration::Calibration calibration;
  Motor motors[4];
  position_control::Move positionMove;
  bool position=false;
  bool canReady = false, ready = false, fault = false;
  bool velocity = false;
  int16_t targets[3] = {}, outputs[3] = {};
  int peakTargets[3] = {}, peakOutputs[3] = {};
  Phase phase = Phase::Idle;
  Reason reason = Reason::Ready;
  uint8_t mask = 0;
  uint32_t runId = 0, remainingMs = 0, errors = 0, failed = 0, tec = 0, rec = 0;
  uint32_t txOk = 0, nonzeroSubmitted = 0, zeroSubmitted = 0, rejected = 0;
  uint32_t rxMissed = 0, rxOverrun = 0, alerts = 0;
  int state = -1;
};
static Snapshot shared;
struct Command { uint8_t mask; uint32_t id, issued; bool heartbeat; bool velocity; int16_t targets[3]; bool localPulse; bool calibrate;
  bool configure;uint8_t motor;uint16_t kpMilli,kiMilli,lowKpMilli;bool configureCurrent;uint16_t currentLimit;
  bool positionStart,positionHeartbeat;int32_t angleCdeg; };
static bool stopRequested = false;
static uint32_t nextRunId = 0;
static bool serialTestUsed = false;
static bool calibrationUsed = false;
static Snapshot snapshot() {
  portENTER_CRITICAL(&snapshotMux);
  Snapshot copy = shared;
  portEXIT_CRITICAL(&snapshotMux);
  return copy;
}
static void requestStop() {
  portENTER_CRITICAL(&snapshotMux);
  stopRequested = true;
  portEXIT_CRITICAL(&snapshotMux);
}
static int16_t signed16(const uint8_t *p) { return static_cast<int16_t>((uint16_t(p[0]) << 8) | p[1]); }
static void canTask(void *) {
  Snapshot s;
  Controller control;
  startup_calibration::Calibration calibration;
  position_control::Move positionMove;
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
  general.rx_queue_len = 1024;
  general.tx_queue_len = 0;
  general.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR |
      TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_RX_FIFO_OVERRUN | TWAI_ALERT_ARB_LOST;
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  s.canReady = twai_driver_install(&general, &timing, &filter) == ESP_OK;
  if (s.canReady) s.canReady = twai_start() == ESP_OK;
  if (!s.canReady) { control.fault = true; control.reason = Reason::CanFault; }
  uint32_t lastPublish = 0;
  bool serialPulse=false;
  uint32_t serialPulseStart=0;
  for (;;) {
    uint32_t now = millis();
    if (s.canReady) {
      twai_message_t frame;
      for (unsigned n = 0; n < 400 && twai_receive(&frame, 0) == ESP_OK; ++n) {
        if (!frame.extd && !frame.rtr && frame.data_length_code == 8 && frame.identifier >= 0x201 && frame.identifier <= 0x204) {
          Motor &m = s.motors[frame.identifier - 0x201];
          m.seen = true; m.lastMs = millis(); ++m.received;
          m.angle = (uint16_t(frame.data[0]) << 8) | frame.data[1];
          m.encoder.observe(m.angle,m.lastMs);
          m.rpm = signed16(frame.data + 2); m.current = signed16(frame.data + 4); m.temp = frame.data[6];
          if (control.phase == Phase::Running || control.phase == Phase::Zeroing || calibration.active())
            m.peakRpm = max(m.peakRpm, abs(int(m.rpm)));
        }
      }
    }
    now = millis();
    bool fresh = true, stopped = true;
    for (unsigned i=0;i<3;++i) {const auto &m=s.motors[i];fresh &= m.seen && now-m.lastMs<=50;stopped &= m.rpm==0;}
    twai_status_info_t status = {};
    bool healthy = s.canReady && twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_RUNNING &&
        status.tx_error_counter == 0 && status.rx_error_counter == 0;
    uint32_t alerts = 0;
    if (s.canReady && twai_read_alerts(&alerts, 0) == ESP_OK) {
      s.alerts |= alerts;
      if (alerts & TWAI_ALERT_TX_SUCCESS) ++s.txOk; // Alert observations, not frame count.
      if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF |
                    TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_RX_FIFO_OVERRUN)) {
        healthy = false;
        control.fault = true; control.reason = Reason::CanFault;
      }
    }
    portENTER_CRITICAL(&snapshotMux);
    bool stop = stopRequested;
    stopRequested = false;
    portEXIT_CRITICAL(&snapshotMux);
    if (stop) control.stop(now, Reason::UserStop);
    Command command;
    for (unsigned i = 0; i < 8 && xQueueReceive(commands, &command, 0) == pdTRUE; ++i) {
      if (OBSERVATION_ONLY) continue;
      if(command.configureCurrent) {
        s.configAck=command.id;
        s.configAccepted=!stop && now-command.issued<250 && !calibration.active() &&
            control.configureCurrent(command.currentLimit,fresh && stopped && healthy);
        if(s.configAccepted)++s.configRevision;
        continue;
      }
      if(command.configure) {
        s.configAck=command.id;
        s.configAccepted=!stop && now-command.issued<250 && !calibration.active() &&
            control.configure(command.motor,command.kpMilli,command.kiMilli,fresh && stopped && healthy,command.lowKpMilli);
        if(s.configAccepted)++s.configRevision;
        continue;
      }
      if (command.calibrate) {
        if (!stop && now-command.issued<250 && calibration.start(now,
            fresh && stopped && healthy && !control.fault && control.phase==Phase::Idle)) {
          for(auto &m:s.motors)m.peakRpm=0;
          s.nonzeroSubmitted=s.zeroSubmitted=s.rejected=0;
        }
        continue;
      }
      if (calibration.active()) continue;
      if(command.positionStart) {
        if(!stop && now-command.issued<250 && control.phase==Phase::Running && control.velocity && !control.position &&
            control.runId==command.id && !control.fault && fresh && stopped && healthy &&
            !control.targets[0] && !control.targets[1] && !control.targets[2] && control.currentLimit==CURRENT &&
            positionMove.start(command.angleCdeg,s.motors[0].encoder,now)) {
          control.position=true;control.mask=1;control.lastHeartbeat=command.issued;
          for(auto &pi:control.speed)pi.reset();
        }
        continue;
      }
      if(command.positionHeartbeat) {
        if(!stop && now-command.issued<250 && positionMove.moving && control.position && control.runId==command.id)
          control.lastHeartbeat=command.issued;
        continue;
      }
      if (command.velocity) {
        const bool wasIdle=control.phase==Phase::Idle;
        if (!stop && now-command.issued < VELOCITY_LEASE_MS &&
            control.velocityCommand(command.targets,command.id,command.issued,fresh && stopped && healthy) && wasIdle) {
          for (auto &m : s.motors) {m.peakRpm=0;m.response.reset();}
          memset(s.peakTargets,0,sizeof(s.peakTargets));memset(s.peakOutputs,0,sizeof(s.peakOutputs));
          s.nonzeroSubmitted=s.zeroSubmitted=s.rejected=0;
        }
      } else if (command.heartbeat) control.heartbeat(command.id, command.issued);
      else if (!stop && now - command.issued < 250 && control.start(command.mask, command.id, now, fresh && stopped && healthy)) {
        for (auto &m : s.motors) m.peakRpm = 0;
        memset(s.peakTargets,0,sizeof(s.peakTargets));memset(s.peakOutputs,0,sizeof(s.peakOutputs));
        s.nonzeroSubmitted = s.zeroSubmitted = s.rejected = 0;
        serialPulse=command.localPulse;serialPulseStart=now;
      }
    }
    if (serialPulse) {
      if (control.phase==Phase::Running && !control.velocity && now-serialPulseStart<RUN_MS)
        control.heartbeat(control.runId,now);
      else serialPulse=false;
    }
    const int16_t rpms[3] = {s.motors[0].rpm,s.motors[1].rpm,s.motors[2].rpm};
    if (calibration.active() && fresh && (rpms[0]>MAX_FEEDBACK_RPM || rpms[0]<-MAX_FEEDBACK_RPM ||
        rpms[1]>MAX_FEEDBACK_RPM || rpms[1]<-MAX_FEEDBACK_RPM || rpms[2]>MAX_FEEDBACK_RPM || rpms[2]<-MAX_FEEDBACK_RPM))
      calibration.abort(now,"overspeed");
    control.update(now, fresh, healthy, rpms);
    if(positionMove.moving && (!control.position || control.phase!=Phase::Running))positionMove.abort(reasonName(control.reason));
    positionMove.update(s.motors[0].encoder,rpms[0],now);
    if(control.position) {
      const bool complete=strcmp(positionMove.reason,"complete")==0;
      if(!positionMove.moving)control.stop(now,complete?Reason::Complete:Reason::PositionFault,!complete);
      else {
        control.targets[0]=positionMove.targetRpm;control.targets[1]=control.targets[2]=0;
        // At the goal, old driving torque must not push past the target.
        // Use zero-speed proportional braking while checking the dwell.
        if(positionMove.targetRpm==0)control.speed[0].reset();
      }
    }
    uint8_t data[8];
    const bool calibrating=calibration.active();
    const bool sendFrame=calibrating ? calibration.frame(now,rpms,fresh,healthy,stop,data) : control.frame(now,data,rpms);
    if (!OBSERVATION_ONLY && s.canReady && sendFrame) {
      if (!calibrating && control.velocity && control.phase==Phase::Running)
        for(unsigned i=0;i<3;++i) s.motors[i].response.observe(control.targets[i],rpms[i],now);
      for (unsigned i=0;i<3;++i) {
        s.peakTargets[i]=max(s.peakTargets[i],abs(int(control.targets[i])));
        s.peakOutputs[i]=max(s.peakOutputs[i],abs(int(control.outputs[i])));
      }
      twai_message_t frame = {};
      frame.identifier = 0x200; frame.data_length_code = 8; frame.ss = 1;
      memcpy(frame.data, data, 8);
      // ID4 is observation-only. Enforce its zero command for every mode.
      frame.data[6]=frame.data[7]=0;
      if (twai_transmit(&frame, 0) == ESP_OK) {
        if(calibrating)calibration.accepted();
        bool nonzero=false;
        for (auto byte : data) nonzero |= byte!=0;
        if (nonzero) ++s.nonzeroSubmitted; else ++s.zeroSubmitted;
      } else {
        ++s.rejected; control.stop(now, Reason::TxFailed, true);
        calibration.abort(now,"tx_failed");
      }
    }
    s.ready = !OBSERVATION_ONLY && fresh && stopped && healthy && !control.fault && control.phase == Phase::Idle && !calibration.active();
    s.phase = control.phase; s.reason = control.reason; s.mask = control.mask;
    s.runId = control.runId; s.fault = control.fault;
    s.remainingMs = control.phase == Phase::Running && !control.velocity ? RUN_MS - (now - control.started) : 0;
    s.velocity=control.velocity;
    s.position=control.position;s.positionMove=positionMove;
    memcpy(s.targets,control.targets,sizeof(s.targets));memcpy(s.outputs,control.outputs,sizeof(s.outputs));
    s.calibration=calibration;
    s.currentLimit=control.currentLimit;
    for(unsigned i=0;i<3;++i) {s.kpMilli[i]=control.speed[i].kpMilli;s.kiMilli[i]=control.speed[i].kiMilli;s.lowKpMilli[i]=control.speed[i].lowKpMilli;}
    if(calibration.active()) {
      s.phase=calibration.stage==startup_calibration::Stage::Ramp ? Phase::Running : Phase::Zeroing;
      memset(s.outputs,0,sizeof(s.outputs));
      if(calibration.trial<6)s.outputs[calibration.trial/2]=calibration.output;
    }
    s.errors = status.bus_error_count; s.failed = status.tx_failed_count;
    s.tec = status.tx_error_counter; s.rec = status.rx_error_counter;
    s.rxMissed = status.rx_missed_count; s.rxOverrun = status.rx_overrun_count;
    s.state = s.canReady ? int(status.state) : -1;
    if (now - lastPublish >= 10) {
      portENTER_CRITICAL(&snapshotMux); shared = s; portEXIT_CRITICAL(&snapshotMux);
      lastPublish = now;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
static void jsonResponse(int code, const String &body) {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.send(code, "application/json", body);
}
static bool authorized(bool stop = false) {
  String token = server.header("X-Control-Token");
  if (stop && token.isEmpty()) token = server.arg("token");
  if (token != controlToken) { jsonResponse(403, "{\"error\":\"reload_required\"}"); return false; }
  return true;
}
static bool parseId(const String &value, uint32_t &id) {
  if (value.isEmpty() || value.length() > 10) return false;
  uint64_t n = 0;
  for (unsigned i = 0; i < value.length(); ++i) {
    if (value[i] < '0' || value[i] > '9') return false;
    n = n * 10 + value[i] - '0';
    if (n > UINT32_MAX) return false;
  }
  id = uint32_t(n); return id != 0;
}
static bool parseRunId(uint32_t &id) { return parseId(server.arg("run"),id); }
static bool parseGain(const String &value,uint16_t &gain) {
  // Integer milli-units only: rejects NaN, signs, exponent notation, and overflow.
  if(value=="0") {gain=0;return true;}
  uint32_t n;if(!parseId(value,n) || n>10000)return false;
  gain=uint16_t(n);return true;
}
static bool parseTarget(const String &value, int16_t &rpm) {
  if (value.isEmpty() || value.length()>4) return false;
  unsigned i=value[0]=='-' ? 1 : 0;
  if (i==value.length()) return false;
  int n=0;
  for (;i<value.length();++i) {
    if (value[i]<'0' || value[i]>'9') return false;
    n=n*10+value[i]-'0';if(n>MAX_TARGET_RPM)return false;
  }
  rpm=value[0]=='-' ? -n : n;return true;
}
static bool parseWheelAngle(const String &value,int32_t &angle) {
  if(value.isEmpty() || value.length()>6)return false;
  unsigned i=value[0]=='-'?1:0;if(i==value.length())return false;
  int n=0;for(;i<value.length();++i) {if(value[i]<'0'||value[i]>'9')return false;n=n*10+value[i]-'0';if(n>18000)return false;}
  if(!n)return false;angle=value[0]=='-'?-n:n;return true;
}
static String positionJson(const Snapshot &s) {
  const auto &p=s.positionMove;
  String j="{\"active\":"+String(s.position?"true":"false")+",\"reason\":\""+p.reason+"\",\"requestedCdeg\":"+String(p.requestedCdeg);
  j+=",\"actualCdeg\":"+String(p.actualCdeg)+",\"errorCdeg\":"+String(p.errorCdeg)+",\"targetRpm\":"+String(p.targetRpm);
  return j+",\"maxAngleCdeg\":18000,\"maxRpm\":200,\"maxCurrentRaw\":1000,\"timeoutMs\":6000}";
}
static String id4Json(const Snapshot &s) {
  const auto &m=s.motors[3];
  char counts[32];snprintf(counts,sizeof(counts),"%lld",(long long)m.encoder.counts);
  String j="{\"seen\":"+String(m.seen?"true":"false")+",\"rpm\":"+String(m.rpm)+",\"peakRpm\":"+String(m.peakRpm)+",\"commandRaw\":0";
  j+=",\"angle\":"+String(m.angle)+",\"counts\":"+String(counts)+",\"generation\":"+String(m.encoder.generation);
  j+=",\"initialized\":"+String(m.encoder.initialized?"true":"false")+",\"received\":"+String(m.received);
  j+=",\"ageMs\":"+String(m.seen?millis()-m.lastMs:UINT32_MAX)+",\"currentRaw\":"+String(m.current);
  return j+"}";
}
static String statusJson(bool includeToken=true) {
  Snapshot s = snapshot();
  String j; j.reserve(1700);
  j = "{\"token\":\"" + (includeToken ? controlToken : String("")) + "\",\"phase\":\"" + phaseName(s.phase) + "\",\"reason\":\"" + reasonName(s.reason);
  j += "\",\"ready\":" + String(s.ready ? "true" : "false") + ",\"fault\":" + String(s.fault ? "true" : "false");
  j += ",\"observationOnly\":"+String(OBSERVATION_ONLY?"true":"false")+",\"bootId\":"+String(observationBootId)+",\"uptimeMs\":"+String(millis());
  j += ",\"canReady\":" + String(s.canReady ? "true" : "false") + ",\"state\":" + String(s.state);
  j += ",\"run\":" + String(s.runId) + ",\"mask\":" + String(s.mask) + ",\"remainingMs\":" + String(s.remainingMs);
  j += ",\"velocity\":" + String(s.velocity ? "true" : "false");
  j += ",\"position\":"+positionJson(s)+",\"id4\":"+id4Json(s);
  j += ",\"settings\":{\"ack\":"+String(s.configAck)+",\"accepted\":"+String(s.configAccepted ? "true" : "false");
  j += ",\"revision\":"+String(s.configRevision)+",\"currentLimitRaw\":"+String(s.currentLimit)+",\"minCurrentLimitRaw\":"+String(CURRENT)+",\"maxCurrentLimitRaw\":"+String(MAX_CURRENT)+",\"targetLimitRpm\":"+String(MAX_TARGET_RPM)+",\"feedbackStopRpm\":"+String(MAX_FEEDBACK_RPM)+",\"volatile\":true,\"motors\":[";
  for(unsigned i=0;i<3;++i) {
    if(i)j+=',';
    j+="{\"id\":"+String(i+1)+",\"kpMilli\":"+String(s.kpMilli[i])+",\"kiMilli\":"+String(s.kiMilli[i])+",\"lowKpMilli\":"+String(s.lowKpMilli[i])+'}';
  }
  j+="]}";
  j += ",\"errors\":" + String(s.errors) + ",\"failed\":" + String(s.failed) + ",\"tec\":" + String(s.tec) + ",\"rec\":" + String(s.rec);
  j += ",\"alerts\":" + String(s.alerts) + ",\"rxMissed\":" + String(s.rxMissed) + ",\"rxOverrun\":" + String(s.rxOverrun);
  j += ",\"nonzeroSubmitted\":" + String(s.nonzeroSubmitted) + ",\"zeroSubmitted\":" + String(s.zeroSubmitted);
  j += ",\"rejected\":" + String(s.rejected) + ",\"motors\":[";
  const uint32_t now = millis();
  for (unsigned i = 0; i < 3; ++i) {
    const auto &m = s.motors[i];
    if (i) j += ',';
    j += "{\"id\":" + String(i+1) + ",\"seen\":" + String(m.seen ? "true" : "false") + ",\"ageMs\":";
    j += m.seen ? String(now - m.lastMs) : "null";
    j += ",\"received\":" + String(m.received) + ",\"rpm\":" + String(m.rpm) + ",\"angle\":" + String(m.angle);
    j += ",\"currentRaw\":" + String(m.current) + ",\"temperature\":" + String(m.temp) + ",\"peakRpm\":" + String(m.peakRpm);
    j += ",\"targetRpm\":" + String(s.targets[i]) + ",\"commandRaw\":" + String(s.outputs[i]);
    j += ",\"peakTargetRpm\":" + String(s.peakTargets[i]) + ",\"peakCommandRaw\":" + String(s.peakOutputs[i]);
    const auto &r=m.response;
    j += ",\"response\":{\"targetRpm\":"+String(r.target)+",\"samples\":"+String(r.samples);
    j += ",\"motionConfirmedMs\":"+(r.motionMs<0 ? String("null") : String(r.motionMs));
    j += ",\"reach90ConfirmedMs\":"+(r.reach90Ms<0 ? String("null") : String(r.reach90Ms));
    j += ",\"peakMean100msRpm\":"+String(r.peakMean100);
    j += ",\"meanAbsErrorRpm\":"+(r.samples ? String(r.absoluteError/r.samples) : String("null"))+"}}";
  }
  j += "],\"calibration\":{\"stage\":\"";
  j += startup_calibration::name(s.calibration.stage);
  j += "\",\"reason\":\""+String(s.calibration.reason)+"\",\"trial\":"+String(s.calibration.trial)+",\"results\":[";
  for(unsigned i=0;i<6;++i) {
    const auto &r=s.calibration.results[i];if(i)j+=',';
    j+="{\"id\":"+String(i/2+1)+",\"direction\":"+String(i%2 ? -1 : 1);
    j+=",\"tested\":"+String(r.tested ? "true" : "false")+",\"detected\":"+String(r.detected ? "true" : "false");
    j+=",\"commandRaw\":"+String(r.command)+",\"rpm\":"+String(r.rpm)+",\"elapsedMs\":"+String(r.elapsedMs)+'}';
  }
  j += "]},\"wifi\":{\"txPowerQuarterDbm\":"+String(txPowerQuarterDbm)+",\"maxPowerConfigured\":"+String(txPowerConfigured?"true":"false")+",\"udpReady\":"+String(udpReady?"true":"false")+",\"udpRejected\":"+String(udpRejected)+"}}";return j;
}
// Keep status below the IPv4 UDP fragmentation threshold. Full diagnostics
// remain available over HTTP/serial; active control never waits on HTTP.
static String udpStatus(uint32_t nonce) {
  const auto s=snapshot();String j;j.reserve(1300);
  j="{\"udpVersion\":1,\"nonce\":"+String(nonce)+",\"token\":\""+controlToken+"\",\"uptimeMs\":"+String(millis());
  j+=",\"ready\":"+String(s.ready?"true":"false")+",\"canReady\":"+String(s.canReady?"true":"false")+",\"fault\":"+String(s.fault?"true":"false");
  j+=",\"phase\":\""+String(phaseName(s.phase))+"\",\"reason\":\""+reasonName(s.reason)+"\",\"run\":"+String(s.runId)+",\"velocity\":"+String(s.velocity?"true":"false");
  j+=",\"position\":"+positionJson(s)+",\"id4\":"+id4Json(s);
  j+=",\"errors\":"+String(s.errors)+",\"failed\":"+String(s.failed)+",\"tec\":"+String(s.tec)+",\"rec\":"+String(s.rec);
  j+=",\"wifi\":{\"txPowerQuarterDbm\":"+String(txPowerQuarterDbm)+",\"maxPowerConfigured\":"+String(txPowerConfigured?"true":"false")+",\"udpRejected\":"+String(udpRejected)+"}";
  j+=",\"settings\":{\"ack\":"+String(s.configAck)+",\"accepted\":"+String(s.configAccepted?"true":"false")+",\"revision\":"+String(s.configRevision)+",\"currentLimitRaw\":"+String(s.currentLimit)+",\"minCurrentLimitRaw\":"+String(CURRENT)+",\"maxCurrentLimitRaw\":"+String(MAX_CURRENT)+",\"targetLimitRpm\":"+String(MAX_TARGET_RPM)+",\"feedbackStopRpm\":"+String(MAX_FEEDBACK_RPM)+",\"volatile\":true,\"motors\":[";
  for(unsigned i=0;i<3;++i) {if(i)j+=',';j+="{\"id\":"+String(i+1)+",\"kpMilli\":"+String(s.kpMilli[i])+",\"kiMilli\":"+String(s.kiMilli[i])+",\"lowKpMilli\":"+String(s.lowKpMilli[i])+'}';}
  j+=" ]},\"motors\":[";
  const uint32_t now=millis();
  for(unsigned i=0;i<3;++i) {const auto &m=s.motors[i];if(i)j+=',';
    j+="{\"id\":"+String(i+1)+",\"seen\":"+String(m.seen?"true":"false")+",\"ageMs\":"+String(m.seen?now-m.lastMs:0xffffffffu)+",\"rpm\":"+String(m.rpm)+",\"commandRaw\":"+String(s.outputs[i])+'}';}
  return j+"]}";
}
static void serviceUdp() {
  for(unsigned n=0;n<8;++n) {
    int size=udp.parsePacket();if(size<=0)return;
    const auto ip=udp.remoteIP();const uint16_t port=udp.remotePort();
    if(size>=180) {udp.flush();++udpRejected;continue;}
    char line[180]={};const int got=udp.read(line,size);
    if(got!=size || memchr(line,0,size) || strstr(line,",,") || line[size-1]==',') {++udpRejected;continue;}
    char *parts[10]={},*save=nullptr;unsigned count=0;
    for(char *p=strtok_r(line,",",&save);p&&count<10;p=strtok_r(nullptr,",",&save))parts[count++]=p;
    uint32_t nonce;
    if(count==2 && strcmp(parts[0],"Q")==0 && parseId(String(parts[1]),nonce)) {
      String j=udpStatus(nonce);
      if(j.length()<=1400) {udp.beginPacket(ip,port);udp.write((const uint8_t*)j.c_str(),j.length());udp.endPacket();}
      continue;
    }
    uint32_t run,seq,expires;
    if((count!=5 && count!=6 && count!=8) || controlToken!=parts[1] || !parseId(String(parts[2]),run) || !parseId(String(parts[3]),seq) || !parseId(String(parts[4]),expires)) {++udpRejected;continue;}
    const auto s=snapshot();const uint32_t now=millis();
    if(count==5 && strcmp(parts[0],"S")==0 && udpGuard.stop(run,seq,expires,now,uint32_t(ip),s.runId)) {
      requestStop();String reply="{\"stopAccepted\":true,\"seq\":"+String(seq)+'}';
      udp.beginPacket(ip,port);udp.print(reply);udp.endPacket();continue;
    }
    Command cmd={};cmd.id=run;cmd.issued=now;
    if((count==6 && strcmp(parts[0],"A")==0) || (count==5 && strcmp(parts[0],"H")==0)) {
      if((count==6 && !parseWheelAngle(String(parts[5]),cmd.angleCdeg)) ||
          !udpGuard.velocity(run,seq,expires,now,uint32_t(ip),false,false,s.runId,s.phase==Phase::Running)) {++udpRejected;continue;}
      cmd.positionStart=count==6;cmd.positionHeartbeat=count==5;
      if(xQueueSend(commands,&cmd,0)!=pdTRUE) {++udpRejected;requestStop();}
      continue;
    }
    cmd.velocity=true;
    if(count!=8 || strcmp(parts[0],"V")!=0 || !parseTarget(String(parts[5]),cmd.targets[0]) || !parseTarget(String(parts[6]),cmd.targets[1]) || !parseTarget(String(parts[7]),cmd.targets[2])) {++udpRejected;continue;}
    const bool neutral=!cmd.targets[0]&&!cmd.targets[1]&&!cmd.targets[2];
    if(!udpGuard.velocity(run,seq,expires,now,uint32_t(ip),neutral,s.ready,s.runId,s.phase==Phase::Running)) {++udpRejected;continue;}
    if(xQueueSend(commands,&cmd,0)!=pdTRUE) {++udpRejected;requestStop();}
  }
}
static void serveStatus() {jsonResponse(200,statusJson());}
static void serialPulseTest() {
  if(serialTestUsed)return;
  serialTestUsed=true;
  if(!snapshot().ready)return;
  if(++nextRunId==0)++nextRunId;
  Command cmd={};cmd.mask=7;cmd.id=nextRunId;cmd.issued=millis();cmd.localPulse=true;
  xQueueSend(commands,&cmd,0);
}
static void serialVelocity(char *line) {
  if(strncmp(line,"A,",2)==0 || strncmp(line,"H,",2)==0) {
    if(strstr(line,",,") || line[strlen(line)-1]==',')return;
    char *parts[4]={},*save=nullptr;unsigned count=0;
    for(char *p=strtok_r(line,",",&save);p&&count<4;p=strtok_r(nullptr,",",&save))parts[count++]=p;
    Command cmd={};cmd.issued=millis();
    if(count<2 || !parseId(String(parts[1]),cmd.id))return;
    if(strcmp(parts[0],"A")==0 && count==3 && parseWheelAngle(String(parts[2]),cmd.angleCdeg))cmd.positionStart=true;
    else if(strcmp(parts[0],"H")==0 && count==2)cmd.positionHeartbeat=true;
    else return;
    xQueueSend(commands,&cmd,0);return;
  }
  if(strncmp(line,"C,",2)==0) {
    // C,<request>,<currentLimit>: stopped-only, volatile, no actuation.
    if(strstr(line,",,") || line[strlen(line)-1]==',')return;
    char *parts[4]={},*save=nullptr;unsigned count=0;
    for(char *p=strtok_r(line,",",&save);p&&count<4;p=strtok_r(nullptr,",",&save))parts[count++]=p;
    Command cmd={};cmd.configureCurrent=true;cmd.issued=millis();uint32_t limit;
    if(count!=3 || !parseId(String(parts[1]),cmd.id) || !parseId(String(parts[2]),limit) || limit<CURRENT || limit>MAX_CURRENT)return;
    cmd.currentLimit=uint16_t(limit);xQueueSend(commands,&cmd,0);return;
  }
  if(strncmp(line,"P,",2)==0) {
    // P,<request>,<motor>,<kpMilli>,<kiMilli>[,<lowKpMilli>]; no actuation or persistence.
    if(strstr(line,",,") || line[strlen(line)-1]==',')return;
    char *parts[7]={},*save=nullptr;unsigned count=0;
    for(char *p=strtok_r(line,",",&save);p&&count<7;p=strtok_r(nullptr,",",&save))parts[count++]=p;
    Command cmd={};cmd.configure=true;cmd.issued=millis();uint32_t motor;
    if((count!=5 && count!=6) || !parseId(String(parts[1]),cmd.id) || !parseId(String(parts[2]),motor) || motor>3 ||
        !parseGain(String(parts[3]),cmd.kpMilli) || !parseGain(String(parts[4]),cmd.kiMilli) ||
        (count==6 && !parseGain(String(parts[5]),cmd.lowKpMilli)))return;
    cmd.motor=motor;xQueueSend(commands,&cmd,0);return;
  }
  if(strncmp(line,"V,",2)!=0 || strstr(line,",,") || line[strlen(line)-1]==',')return;
  char *parts[6]={},*save=nullptr;unsigned count=0;
  for(char *p=strtok_r(line,",",&save);p&&count<6;p=strtok_r(nullptr,",",&save))parts[count++]=p;
  if(count!=5 || strcmp(parts[0],"V")!=0)return;
  Command cmd={};cmd.velocity=true;cmd.issued=millis();
  if(!parseId(String(parts[1]),cmd.id))return;
  for(unsigned i=0;i<3;++i)if(!parseTarget(String(parts[i+2]),cmd.targets[i]))return;
  xQueueSend(commands,&cmd,0);
}
static void serviceSerial() {
  static char line[80];static unsigned length=0;static bool overflow=false;
  for(unsigned n=0;n<256&&Serial.available();++n) {
    char c=Serial.read();
    if(c=='z'){requestStop();length=0;overflow=false;continue;}
    if(c=='t'&&length==0&&!overflow){serialPulseTest();continue;}
    if(c=='c'&&length==0&&!overflow) {
      if(!calibrationUsed) {
        calibrationUsed=true;
        if(snapshot().ready) {
          Command cmd={};cmd.calibrate=true;cmd.issued=millis();xQueueSend(commands,&cmd,0);
        }
      }
      continue;
    }
    if(c=='\r')continue;
    if(c=='\n') {
      if(!overflow&&length){line[length]=0;serialVelocity(line);}
      length=0;overflow=false;
    } else if(!overflow) {
      if(length+1<sizeof(line))line[length++]=c;else overflow=true;
    }
  }
}
void setup() {
  Serial.begin(115200);
  commands = xQueueCreate(8, sizeof(Command));
  if (!commands) {
    Serial.println("CAN task initialization failed. No motor commands enabled.");
    while (true) delay(1000);
  }
  char token[33];
  snprintf(token, sizeof(token), "%08lx%08lx%08lx%08lx", (unsigned long)esp_random(), (unsigned long)esp_random(), (unsigned long)esp_random(), (unsigned long)esp_random());
  controlToken = token;
  observationBootId = esp_random();
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  apReady = WiFi.softAP(AP_SSID, AP_PASSWORD, 6, false, 4);
  // Use driver-supported maximum without overriding country/channel limits.
  if(apReady) {
    txPowerConfigured=esp_wifi_set_max_tx_power(84)==ESP_OK;
    esp_wifi_get_max_tx_power(&txPowerQuarterDbm);
    WiFi.setSleep(false);
    udpReady=udp.begin(4210)!=0;
  }
  const char *headers[] = {"X-Control-Token"}; server.collectHeaders(headers, 1);
  server.on("/", HTTP_GET, [] {
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("X-Frame-Options", "DENY");
    server.send_P(200, "text/html; charset=utf-8", WEB_UI);
  });
  server.on("/api/status", HTTP_GET, serveStatus);
  server.on("/api/config", HTTP_POST, [] {
    if(!authorized())return;
    Command cmd={};cmd.configure=true;cmd.issued=millis();uint32_t motor;
    if(!parseId(server.arg("request"),cmd.id) || !parseId(server.arg("motor"),motor) || motor>3 ||
        !parseGain(server.arg("kpMilli"),cmd.kpMilli) || !parseGain(server.arg("kiMilli"),cmd.kiMilli) ||
        (server.hasArg("lowKpMilli") && !parseGain(server.arg("lowKpMilli"),cmd.lowKpMilli))) {
      jsonResponse(400,"{\"error\":\"invalid_settings\"}");return;
    }
    if(!snapshot().ready || uxQueueMessagesWaiting(commands)) {jsonResponse(409,"{\"error\":\"not_ready\"}");return;}
    cmd.motor=motor;
    if(xQueueSend(commands,&cmd,0)!=pdTRUE) {jsonResponse(503,"{\"error\":\"busy\"}");return;}
    jsonResponse(202,"{\"request\":"+String(cmd.id)+'}');
  });
  server.on("/api/current-limit", HTTP_POST, [] {
    if(!authorized())return;
    Command cmd={};cmd.configureCurrent=true;cmd.issued=millis();uint32_t limit;
    if(!parseId(server.arg("request"),cmd.id) || !parseId(server.arg("limit"),limit) || limit<CURRENT || limit>MAX_CURRENT) {
      jsonResponse(400,"{\"error\":\"invalid_current_limit\"}");return;
    }
    if(!snapshot().ready || uxQueueMessagesWaiting(commands)) {jsonResponse(409,"{\"error\":\"not_ready\"}");return;}
    cmd.currentLimit=uint16_t(limit);
    if(xQueueSend(commands,&cmd,0)!=pdTRUE) {jsonResponse(503,"{\"error\":\"busy\"}");return;}
    jsonResponse(202,"{\"request\":"+String(cmd.id)+'}');
  });
  server.on("/api/start", HTTP_POST, [] {
    if (!authorized()) return;
    String value = server.arg("mask");
    if (value.length() != 1 || value[0] < '1' || value[0] > '7' || server.arg("confirm") != "yes") {
      jsonResponse(400, "{\"error\":\"invalid_request\"}"); return;
    }
    if (!snapshot().ready || uxQueueMessagesWaiting(commands) != 0) {
      jsonResponse(409, "{\"error\":\"not_ready\"}"); return;
    }
    if (++nextRunId == 0) ++nextRunId;
    Command cmd = {uint8_t(value[0]-'0'), nextRunId, millis(), false};
    if (xQueueSend(commands, &cmd, 0) != pdTRUE) { jsonResponse(503, "{\"error\":\"busy\"}"); return; }
    jsonResponse(202, "{\"run\":" + String(nextRunId) + '}');
  });
  server.on("/api/heartbeat", HTTP_POST, [] {
    if (!authorized()) return;
    uint32_t id;
    if (!parseRunId(id)) { jsonResponse(400, "{\"error\":\"invalid_run\"}"); return; }
    auto s = snapshot();
    if (s.phase != Phase::Running || s.velocity || s.runId != id) { jsonResponse(409, "{\"error\":\"inactive_run\"}"); return; }
    Command cmd = {0, id, millis(), true};
    if (xQueueSend(commands, &cmd, 0) != pdTRUE) { jsonResponse(503, "{\"error\":\"busy\"}"); return; }
    jsonResponse(200, "{\"ok\":true}");
  });
  server.on("/api/velocity", HTTP_POST, [] {
    if (!authorized()) return;
    Command cmd = {};cmd.velocity=true;cmd.issued=millis();
    if (!parseTarget(server.arg("t1"),cmd.targets[0]) || !parseTarget(server.arg("t2"),cmd.targets[1]) ||
        !parseTarget(server.arg("t3"),cmd.targets[2])) {
      jsonResponse(400,"{\"error\":\"invalid_target\"}");return;
    }
    const auto s=snapshot();
    if(server.arg("run")=="0") {
      if (!s.ready || cmd.targets[0] || cmd.targets[1] || cmd.targets[2] || uxQueueMessagesWaiting(commands)!=0) {
        jsonResponse(409,"{\"error\":\"neutral_start_required\"}");return;
      }
      if (++nextRunId==0)++nextRunId;
      cmd.id=nextRunId;
    } else if(!parseRunId(cmd.id) || !s.velocity || s.phase!=Phase::Running || s.runId!=cmd.id) {
      jsonResponse(409,"{\"error\":\"inactive_run\"}");return;
    }
    if (xQueueSend(commands,&cmd,0)!=pdTRUE) {jsonResponse(503,"{\"error\":\"busy\"}");return;}
    jsonResponse(202,"{\"run\":"+String(cmd.id)+'}');
  });
  server.on("/api/stop", HTTP_POST, [] {
    if (!authorized(true)) return;
    requestStop(); jsonResponse(200, "{\"ok\":true}");
  });
  server.onNotFound([] { jsonResponse(404, "{\"error\":\"not_found\"}"); });
  if (apReady) server.begin();
  // Wi-Fi initialization may block flash/interrupt servicing. Begin CAN only
  // after the radio is initialized, rather than dropping incoming frames at boot.
  if (xTaskCreatePinnedToCore(canTask, "can-control", 6144, nullptr, 3, nullptr, 1) != pdPASS) {
    server.stop();
    Serial.println("CAN task initialization failed. No motor commands enabled.");
    while (true) delay(1000);
  }
  Serial.printf("C620 WEB: AP=%s IP=%s SSID=%s; no data TX on boot\n", apReady ? "OK" : "FAIL", WiFi.softAPIP().toString().c_str(), AP_SSID);
}
void loop() {
  if(udpReady)serviceUdp();
  if (apReady) server.handleClient();
  serviceSerial();
  static uint32_t lastReport = 0;
  static String pendingReport;
  static unsigned reportOffset=0;
  if (millis() - lastReport >= 1000) {
    lastReport = millis();
    if(pendingReport.isEmpty()) {
      pendingReport="STATUS "+statusJson(false)+"\n";reportOffset=0;
    }
  }
  // Never wait for a full UART buffer: service incoming control/stop commands
  // between chunks, even as telemetry grows. CAN task timing stays independent.
  if(!pendingReport.isEmpty()) {
    const unsigned room=Serial.availableForWrite();
    const unsigned left=pendingReport.length()-reportOffset;
    const unsigned count=room<left ? room : left;
    if(count)reportOffset+=Serial.write(reinterpret_cast<const uint8_t *>(pendingReport.c_str()+reportOffset),count);
    if(reportOffset==pendingReport.length())pendingReport="";
  }
  delay(1);
}

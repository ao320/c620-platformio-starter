#pragma once
#include <stdint.h>
#include <string.h>

namespace motor_control {
constexpr uint16_t CURRENT = 1000;
constexpr uint16_t MAX_CURRENT = 6000;
constexpr uint32_t RUN_MS = 1000, ZERO_MS = 1000, PERIOD_MS = 10, LEASE_MS = 500;
constexpr int MAX_TARGET_RPM = 900;
// User-specified overspeed threshold. Target and current caps are independent.
constexpr int MAX_FEEDBACK_RPM = 2000;
constexpr uint32_t VELOCITY_LEASE_MS = 250;
// Validated lifted-wheel profile, 2026-10-27. Boot remains idle/no data TX.
constexpr uint16_t DEFAULT_KP[3] = {3000,6000,4000};
constexpr uint16_t DEFAULT_KI[3] = {6000,8000,2000};
constexpr uint16_t DEFAULT_LOW_KP[3] = {0,0,7000};
struct SpeedPI {
  uint16_t kpMilli = 1500, kiMilli = 1000;
  // Optional extra proportional gain at low targets. Blend back to kpMilli
  // between 180 and 260 RPM so stick movement cannot cause a gain step.
  uint16_t lowKpMilli = 0;
  float integral = 0;
  void reset() { integral = 0; }
  float proportionalGain(int16_t target) const {
    int magnitude=target<0 ? -int(target) : int(target);
    if (!lowKpMilli || magnitude>=260) return kpMilli*0.001f;
    if (magnitude<=180) return lowKpMilli*0.001f;
    const float blend=(magnitude-180)*0.0125f;
    return (lowKpMilli+(float(kpMilli)-lowKpMilli)*blend)*0.001f;
  }
  int16_t step(int16_t target, int16_t rpm, uint16_t currentLimit=CURRENT) {
    const float error = float(target) - rpm;
    const float candidate = integral + error * (kiMilli * 0.00001f);
    const float kp=proportionalGain(target);
    float raw = kp * error + candidate;
    // Do not wind up while saturated in the same direction as the error.
    if (!((raw > currentLimit && error > 0) || (raw < -int(currentLimit) && error < 0))) integral = candidate;
    raw = kp * error + integral;
    if (raw > currentLimit) raw = currentLimit;
    if (raw < -int(currentLimit)) raw = -int(currentLimit);
    return static_cast<int16_t>(raw);
  }
};
enum class Phase { Idle, Running, Zeroing };
enum class Reason { Ready, Running, Complete, UserStop, BrowserLost, FeedbackLost, CanFault, TxFailed, Overspeed, PositionFault };
inline const char *reasonName(Reason r) {
  switch (r) {
    case Reason::Ready: return "ready";
    case Reason::Running: return "running";
    case Reason::Complete: return "complete";
    case Reason::UserStop: return "user_stop";
    case Reason::BrowserLost: return "browser_lost";
    case Reason::FeedbackLost: return "feedback_lost";
    case Reason::CanFault: return "can_fault";
    case Reason::TxFailed: return "tx_failed";
    case Reason::Overspeed: return "overspeed";
    case Reason::PositionFault: return "position_fault";
  }
  return "unknown";
}
inline const char *phaseName(Phase p) {
  return p == Phase::Running ? "running" : p == Phase::Zeroing ? "zeroing" : "idle";
}
inline bool due(uint32_t now, uint32_t deadline) { return static_cast<int32_t>(now - deadline) >= 0; }
inline void encode(uint8_t mask, uint8_t data[8], uint16_t currentRaw=CURRENT) {
  memset(data, 0, 8);
  for (unsigned i = 0; i < 3; ++i) if (mask & (1u << i)) {
    data[2*i] = currentRaw >> 8;
    data[2*i+1] = currentRaw & 0xFF;
  }
}
class Controller {
 public:
  Controller() {
    for(unsigned i=0;i<3;++i) {
      speed[i].kpMilli=DEFAULT_KP[i];speed[i].kiMilli=DEFAULT_KI[i];
      speed[i].lowKpMilli=DEFAULT_LOW_KP[i];
    }
  }
  Phase phase = Phase::Idle;
  Reason reason = Reason::Ready;
  bool fault = false;
  bool velocity = false;
  bool position = false;
  int16_t targets[3] = {}, outputs[3] = {};
  SpeedPI speed[3];
  uint16_t currentLimit = CURRENT;
  uint8_t mask = 0;
  uint32_t runId = 0, started = 0, zeroStarted = 0, lastHeartbeat = 0, nextSend = 0;
  bool configure(unsigned motor, uint16_t kp, uint16_t ki, bool ready, uint16_t lowKp=0) {
    if (!ready || fault || phase!=Phase::Idle || motor<1 || motor>3 || kp>10000 || ki>10000 || lowKp>10000) return false;
    speed[motor-1].kpMilli=kp;speed[motor-1].kiMilli=ki;
    speed[motor-1].lowKpMilli=lowKp;
    for(auto &pi:speed)pi.reset();
    return true;
  }
  bool configureCurrent(uint16_t requested, bool ready) {
    if (!ready || fault || phase!=Phase::Idle || requested<CURRENT || requested>MAX_CURRENT) return false;
    currentLimit=requested;
    for(auto &pi:speed)pi.reset();
    return true;
  }
  bool start(uint8_t requested, uint32_t id, uint32_t now, bool ready) {
    if (!ready || fault || phase != Phase::Idle || requested == 0 || requested > 7 || id == 0 || id == runId) return false;
    mask = requested; runId = id; started = lastHeartbeat = nextSend = now;
    velocity = false;
    phase = Phase::Running; reason = Reason::Running;
    return true;
  }
  void heartbeat(uint32_t id, uint32_t now) {
    if (phase == Phase::Running && !velocity && id == runId) lastHeartbeat = now;
  }
  bool velocityCommand(const int16_t requested[3], uint32_t id, uint32_t now, bool ready) {
    if(position)return false;
    for (unsigned i=0;i<3;++i) if (requested[i] < -MAX_TARGET_RPM || requested[i] > MAX_TARGET_RPM) return false;
    if (phase == Phase::Idle) {
      // A new session must begin at neutral; held sticks never start motion.
      if (requested[0] || requested[1] || requested[2] || !start(7,id,now,ready)) return false;
      velocity = true;
      for (auto &pi : speed) pi.reset();
    }
    if (fault || phase != Phase::Running || !velocity || id != runId) return false;
    memcpy(targets,requested,sizeof(targets)); lastHeartbeat=now;
    return true;
  }
  void stop(uint32_t now, Reason why, bool latchFault = false) {
    fault = fault || latchFault;
    reason = why;
    mask = 0;
    position=false;
    memset(targets,0,sizeof(targets)); memset(outputs,0,sizeof(outputs));
    for (auto &pi : speed) pi.reset();
    if (phase != Phase::Zeroing) { phase = Phase::Zeroing; zeroStarted = nextSend = now; }
  }
  void update(uint32_t now, bool feedbackFresh, bool canHealthy, const int16_t *rpms=nullptr) {
    if (phase == Phase::Running) {
      if (!canHealthy) stop(now, Reason::CanFault, true);
      else if (!feedbackFresh) stop(now, Reason::FeedbackLost, true);
      else if (rpms && (rpms[0]>MAX_FEEDBACK_RPM || rpms[0]<-MAX_FEEDBACK_RPM ||
                        rpms[1]>MAX_FEEDBACK_RPM || rpms[1]<-MAX_FEEDBACK_RPM ||
                        rpms[2]>MAX_FEEDBACK_RPM || rpms[2]<-MAX_FEEDBACK_RPM)) stop(now, Reason::Overspeed, true);
      else if (now - lastHeartbeat >= (velocity ? VELOCITY_LEASE_MS : LEASE_MS)) stop(now, Reason::BrowserLost);
      else if (!velocity && now - started >= RUN_MS) stop(now, Reason::Complete);
    }
    if (phase == Phase::Zeroing && now - zeroStarted >= ZERO_MS) phase = Phase::Idle;
  }
  bool frame(uint32_t now, uint8_t data[8], const int16_t *rpms = nullptr) {
    if (phase == Phase::Idle || !due(now, nextSend)) return false;
    nextSend = now + PERIOD_MS;
    if (phase == Phase::Running && velocity) {
      memset(data,0,8);
      if (!rpms) { stop(now,Reason::FeedbackLost,true); return true; }
      const bool neutral = targets[0]==0 && targets[1]==0 && targets[2]==0;
      for (unsigned i=0;i<3;++i) {
        if ((neutral && !position) || !(mask & (1u<<i))) { speed[i].reset(); outputs[i]=0; }
        else outputs[i]=speed[i].step(targets[i],rpms[i],currentLimit);
        const uint16_t bits=static_cast<uint16_t>(outputs[i]);
        data[2*i]=bits>>8;data[2*i+1]=bits&255;
      }
    } else {
      // Apply the configured cap to the Web UI's direct-current 1s pulse too.
      encode(phase == Phase::Running ? mask : 0,data,currentLimit);
      for (unsigned i=0;i<3;++i) outputs[i]=(phase==Phase::Running && (mask & (1u<<i))) ? currentLimit : 0;
    }
    return true;
  }
};
}

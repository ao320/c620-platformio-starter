#include <Arduino.h>
#include <driver/twai.h>
#include <esp_system.h>
#include "../../src/arm_position_control.h"
#include "../../src/arm_trace.h"
#include "../../src/arm_auto_trial.h"
#ifndef ARM_ENABLE_ACTUATION
#define ARM_ENABLE_ACTUATION 0
#endif
#ifndef ARM_ENABLE_GRAVITY_PROFILE
#define ARM_ENABLE_GRAVITY_PROFILE 0
#endif
#ifndef ARM_ENABLE_STANDALONE
#define ARM_ENABLE_STANDALONE 0
#endif
// Dedicated arm image: no chassis command endpoints, no boot data TX.
static arm_position::Control control;
static position_control::Encoder encoder;
static arm_trace::Recorder trace;
static arm_auto::Trial autoTrial;
static int16_t rpm=0,feedbackCurrent=0;
static uint16_t angle=0;
static uint8_t temperature=0;
static bool canReady=false,seen=false,healthy=true,submitted=false;
static uint32_t bootId=0,rx=0,errors=0,tx=0,failed=0,lastReport=0,lastSend=0,lastCommand=0;
static uint32_t stopUntil=0;
static int peakRpm=0,peakCommand=0,lastSessionOverspeedRpm=0;
static int tec=0,rec=0;
static uint32_t canFrames=0,otherFrames=0,lastOtherId=0,alertMask=0;
static uint32_t motorFeedbackFrames[4]={0,0,0,0};
static uint32_t lastStandaloneSession=0;
static uint32_t limitAck=0;static bool limitAccepted=false;
static int16_t signed16(const uint8_t *p){return int16_t(uint16_t(p[0])<<8|p[1]);}
static arm_trace::Sample traceSample(){
  arm_trace::Sample s;s.ms=millis();s.position=control.position(encoder);s.target=int32_t(control.target);
  s.targetRpm=control.targetRpm;s.rpm=rpm;s.command=control.output;s.current=feedbackCurrent;
  s.integral=control.speed.integral;s.feedbackAgeMs=seen?s.ms-encoder.lastMs:UINT32_MAX;return s;
}
static void stop(const char *reason){
  if(autoTrial.guard.active)autoTrial.guard.fail(reason);
  const auto before=traceSample();control.stop(reason);stopUntil=millis()+1000;
  trace.stop(before,traceSample(),reason);
}
static void input(const String &line){
  const uint32_t now=millis();
  if(line=="ARM,STOP"){if(control.active)stop("manual_stop");else if(submitted)stopUntil=now+1000;return;}
  if(line=="ARM,HOME"){if(!submitted||!control.active)control.home(encoder,rpm,now);return;}
  if(!ARM_ENABLE_ACTUATION)return;
  unsigned long id=0;int direction=0;char trailing=0;
  unsigned long value=0;
  if(sscanf(line.c_str(),"ARM,LIMIT,%lu,%lu%c",&id,&value,&trailing)==2){
    limitAck=uint32_t(id);limitAccepted=healthy&&seen&&now-encoder.lastMs<=20&&int32_t(now-stopUntil)>=0&&value>=100&&value<=6000&&control.configureCurrent(uint16_t(value),rpm);return;
  }
  if(sscanf(line.c_str(),"ARM,AUTO,%lu%c",&id,&trailing)==1){
    if(int32_t(now-stopUntil)>=0 && control.overspeedRpm==1000 && temperature<60 &&
       autoTrial.start(control,encoder,rpm,healthy&&seen&&canReady,now,uint32_t(id))){
      lastCommand=now;peakRpm=peakCommand=0;lastSessionOverspeedRpm=control.overspeedRpm;
      trace.begin(control.session,60);
    }
  }else if(sscanf(line.c_str(),"ARM,AUTOLEASE,%lu%c",&id,&trailing)==1){
    autoTrial.renew(uint32_t(id),now);
  }else if(sscanf(line.c_str(),"ARM,ENABLE1000,%lu%c",&id,&trailing)==1){
    if(control.overspeedRpm==1000&&healthy&&temperature<60&&control.enable(encoder,rpm,now,uint32_t(id))){lastCommand=now;peakRpm=peakCommand=0;lastSessionOverspeedRpm=control.overspeedRpm;trace.begin(control.session);}
  }else if(sscanf(line.c_str(),"ARM,ENABLE,%lu%c",&id,&trailing)==1){
    if(healthy&&temperature<60&&control.enable(encoder,rpm,now,uint32_t(id))){lastCommand=now;peakRpm=peakCommand=0;lastSessionOverspeedRpm=control.overspeedRpm;trace.begin(control.session);}
  }else if(sscanf(line.c_str(),"ARM,COMPARE,%lu%c",&id,&trailing)==1){
    if(healthy&&temperature<60&&control.startComparison(encoder,rpm,now,uint32_t(id))){lastCommand=now;peakRpm=peakCommand=0;lastSessionOverspeedRpm=control.overspeedRpm;trace.begin(control.session);}
  }else if(sscanf(line.c_str(),"ARM,FREE,%lu%c",&id,&trailing)==1){
    if(ARM_ENABLE_STANDALONE&&id!=lastStandaloneSession&&int32_t(now-stopUntil)>=0&&healthy&&temperature<60&&control.startStandalone(encoder,rpm,now,uint32_t(id))){lastStandaloneSession=uint32_t(id);lastCommand=now;peakRpm=peakCommand=0;lastSessionOverspeedRpm=control.overspeedRpm;trace.begin(control.session);}
  }else if(sscanf(line.c_str(),"ARM,MOVE,%lu,%d%c",&id,&direction,&trailing)==2){
    if(!autoTrial.guard.active&&control.command(uint32_t(id),direction,now))lastCommand=now;
  }
}
static String status(){
  char counts[32],pos[32];snprintf(counts,sizeof(counts),"%lld",(long long)encoder.counts);
  snprintf(pos,sizeof(pos),"%lld",(long long)control.position(encoder));
  String j="{\"armFirmware\":1,\"actuationEnabled\":"+String(ARM_ENABLE_ACTUATION?"true":"false")+",\"bootId\":"+String(bootId);
  j+=",\"resetReasonCode\":"+String(int(esp_reset_reason()))+",\"freeHeapBytes\":"+String(ESP.getFreeHeap());
  j+=",\"canReady\":"+String(canReady?"true":"false")+",\"healthy\":"+String(healthy?"true":"false");
  j+=",\"homed\":"+String(control.homed?"true":"false")+",\"active\":"+String(control.active?"true":"false")+",\"fault\":"+String(control.fault?"true":"false");
  j+=",\"reason\":\""+String(control.reason)+"\",\"session\":"+String(control.session)+",\"uptimeMs\":"+String(millis());
  j+=",\"autoActive\":"+String(autoTrial.guard.active?"true":"false")+",\"autoLatched\":"+String(autoTrial.guard.latched?"true":"false")+",\"autoLeg\":"+String(autoTrial.guard.legs)+",\"autoReason\":\""+String(autoTrial.guard.reason)+"\"";
  j+=",\"positionCounts\":"+String(pos)+",\"targetCounts\":"+String(control.target,1)+",\"spanCounts\":"+String(arm_position::SPAN);
  j+=",\"marginCounts\":"+String(arm_position::MARGIN)+",\"errors\":"+String(errors)+",\"failed\":"+String(failed)+",\"tx\":"+String(tx);
  j+=",\"maxJogUpRpm\":"+String(control.gravityAware?arm_position::JOG_UP_MAX_RPM:arm_position::MAX_RPM)+",\"jogUpCountsPerSecond\":"+String(control.gravityAware?0:arm_position::JOG_COUNTS_PER_SECOND);
  j+=",\"jogUpStepCounts\":0,\"jogUpSettleMs\":0";
  j+=",\"manualUpperHoldCounts\":"+String(arm_position::MANUAL_UPPER_HOLD)+",\"manualUpperHold\":"+String(control.manualUpperHold?"true":"false");
  j+=",\"jogLimitEnabled\":false,\"jogSessionMs\":0,\"comparisonSessionMs\":2000,\"maxManualCurrentLimitRaw\":6000,\"currentSlewRawPerSecond\":"+String(arm_position::UP_CURRENT_RISE_PER_SECOND)+",\"highCurrentProfile\":"+String(control.currentLimit>2000?"true":"false");
  j+=",\"controlProfile\":\""+String(control.gravityAware?"gravity_velocity_jog_hold_brake_v2":"fixed_gain_soft_start_v1")+"\",\"speedKpMilli\":"+String(control.speed.kpMilli)+",\"speedKiMilli\":"+String(control.speed.kiMilli);
  j+=",\"lastSessionOverspeedRpm\":"+String(lastSessionOverspeedRpm);
  j+=",\"overspeedRpm\":"+String(control.overspeedRpm);
  j+=",\"accelerationGuardEnabled\":"+String(control.accelerationGuardEnabled?"true":"false")+",\"accelerationGuardMs\":"+String(control.accelerationGuardMs)+",\"accelerationEvents\":"+String(control.accelerationEvents);
  j+=",\"uphillIntegralKiMilli\":40000";
  j+=",\"gravityAware\":"+String(control.gravityAware?"true":"false")+",\"maxDescentRpm\":"+String(control.gravityAware?arm_position::MAX_DESCENT_RPM:60);
  j+=",\"brakingKpMilli\":"+String(control.gravityAware?36000:6000)+",\"gravityOnlyDescent\":false,\"manualDescentCurrentLimitRaw\":"+String(arm_position::DESCENT_CURRENT_LIMIT)+",\"manualDescentCurrentRiseRawPerSecond\":"+String(arm_position::DESCENT_CURRENT_RISE_PER_SECOND)+",\"upwardBrakeCurrentLimitRaw\":"+String(arm_position::UPWARD_BRAKE_CURRENT_LIMIT)+",\"upwardBrakeRiseRawPerSecond\":"+String(arm_position::UPWARD_BRAKE_RISE_PER_SECOND)+",\"brakingCurrentSlewLimited\":true";
  j+=",\"targetAccelerationRpmPerSecond\":"+String(control.gravityAware?30:60)+",\"promptTorqueReduction\":true";
  j+=",\"jogUpTargetAccelerationRpmPerSecond\":"+String(control.gravityAware?90:60);
  j+=",\"manualCurrentLimitRaw\":"+String(control.manualLimit)+",\"limitAck\":"+String(limitAck)+",\"limitAccepted\":"+String(limitAccepted?"true":"false");
  j+=",\"standaloneRepeatable\":true,\"zeroing\":"+String(int32_t(stopUntil-millis())>0?"true":"false");
  j+=",\"standaloneSupported\":"+String(ARM_ENABLE_STANDALONE?"true":"false")+",\"standalone\":"+String(control.standalone?"true":"false");
  j+=",\"comparisonSupported\":true,\"comparison\":"+String(control.comparison?"true":"false")+",\"currentLimitRaw\":"+String(control.currentLimit);
  j+=",\"tec\":"+String(tec)+",\"rec\":"+String(rec)+",\"targetRpm\":"+String(control.targetRpm)+",\"peakRpm\":"+String(peakRpm)+",\"peakCommandRaw\":"+String(peakCommand);
  j+=",\"canFrames\":"+String(canFrames)+",\"otherFrames\":"+String(otherFrames)+",\"lastOtherId\":"+String(lastOtherId)+",\"alertMask\":"+String(alertMask)+",\"motorFeedbackFrames\":["+String(motorFeedbackFrames[0])+","+String(motorFeedbackFrames[1])+","+String(motorFeedbackFrames[2])+","+String(motorFeedbackFrames[3])+"]";
  j+=",\"id4\":{\"angle\":"+String(angle)+",\"counts\":"+String(counts)+",\"generation\":"+String(encoder.generation);
  j+=",\"ageMs\":"+String(seen?millis()-encoder.lastMs:UINT32_MAX)+",\"rpm\":"+String(rpm)+",\"commandRaw\":"+String(control.output);
  j+=",\"currentRaw\":"+String(feedbackCurrent)+",\"temperature\":"+String(temperature)+",\"received\":"+String(rx)+"}}";
  j.remove(j.length()-1);
  j+=",\"trace\":{\"version\":1,\"session\":"+String(trace.session)+",\"ready\":"+String(trace.ready?"true":"false");
  j+=",\"reason\":\""+String(trace.reason)+"\",\"nominalPeriodMs\":"+String(trace.periodMs)+",\"offset\":"+String(trace.exported)+",\"total\":"+String(trace.count);
  j+=",\"columns\":[\"ms\",\"positionCounts\",\"targetCounts\",\"targetRpm\",\"rpm\",\"commandRaw\",\"currentRaw\",\"integralRaw\",\"feedbackAgeMs\"],\"samples\":[";
  const unsigned end=trace.ready?min(trace.count,trace.exported+arm_trace::Recorder::CHUNK):trace.exported;
  for(unsigned i=trace.exported;i<end;++i){
    const auto &s=trace.at(i);char position[32];snprintf(position,sizeof(position),"%lld",(long long)s.position);
    if(i>trace.exported)j+=",";
    j+="["+String(s.ms)+","+String(position)+","+String(s.target)+","+String(s.targetRpm)+","+String(s.rpm)+","+String(s.command)+","+String(s.current)+","+String(s.integral,2)+","+String(s.feedbackAgeMs)+"]";
  }
  const bool cycleComplete=trace.ready&&end==trace.count;
  trace.advanceExport(end);j+="],\"complete\":"+String(cycleComplete?"true":"false")+"}}";
  return j;
}
void setup(){
 Serial.begin(115200);bootId=esp_random();control.gravityAware=ARM_ENABLE_GRAVITY_PROFILE;
 twai_general_config_t g=TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21,GPIO_NUM_22,TWAI_MODE_NORMAL);
 g.rx_queue_len=1024;g.tx_queue_len=0;
 g.alerts_enabled=TWAI_ALERT_BUS_ERROR|TWAI_ALERT_BUS_OFF|TWAI_ALERT_RX_QUEUE_FULL|TWAI_ALERT_RX_FIFO_OVERRUN;
 twai_timing_config_t t=TWAI_TIMING_CONFIG_1MBITS();twai_filter_config_t f=TWAI_FILTER_CONFIG_ACCEPT_ALL();
 canReady=twai_driver_install(&g,&t,&f)==ESP_OK;
 if(canReady)canReady=twai_start()==ESP_OK;
 healthy=canReady;
}
void loop(){
 uint32_t now=millis();
 twai_message_t frame;
 if(canReady){
  for(unsigned i=0;i<400&&twai_receive(&frame,0)==ESP_OK;++i){
   ++canFrames;
   if(!frame.extd&&!frame.rtr&&frame.identifier>=0x201&&frame.identifier<=0x204)
     ++motorFeedbackFrames[frame.identifier-0x201];
   else{++otherFrames;lastOtherId=frame.identifier;}
   if(frame.identifier!=0x204||frame.extd||frame.rtr||frame.data_length_code!=8)continue;
   angle=uint16_t(frame.data[0])<<8|frame.data[1];rpm=signed16(frame.data+2);
   feedbackCurrent=signed16(frame.data+4);temperature=frame.data[6];encoder.observe(angle,millis());seen=true;++rx;
  }
  uint32_t alerts=0;twai_read_alerts(&alerts,0);
  if(alerts){++errors;alertMask|=alerts;healthy=false;}
  twai_status_info_t info;
  if(twai_get_status_info(&info)==ESP_OK){tec=info.tx_error_counter;rec=info.rx_error_counter;}
 }
 now=millis();
 static String line;
 for(unsigned i=0;i<96&&Serial.available();++i){
  char c=Serial.read();if(c=='\n'){input(line);line="";}else if(c!='\r'){line+=c;if(line.length()>80)line="";}
 }
 trace.observe(traceSample());
 const bool wasActive=control.active;
 const auto beforeUpdate=traceSample();
 now=millis();
 // Include the sample that triggers overspeed in the recorded peak.
 if(wasActive){const int ar=abs(int(rpm));if(ar>peakRpm)peakRpm=ar;}
 autoTrial.tick(control,encoder,rpm,healthy&&temperature<60,now);
 control.update(encoder,rpm,now,healthy&&temperature<60);
 if(autoTrial.guard.active&&!control.active)autoTrial.guard.fail(control.reason);
 if(wasActive&&!control.active){stopUntil=now+1000;trace.stop(beforeUpdate,traceSample(),control.reason);}
 if(control.active){int ar=rpm<0?-int(rpm):int(rpm),ac=control.output<0?-int(control.output):int(control.output);if(ar>peakRpm)peakRpm=ar;if(ac>peakCommand)peakCommand=ac;}
 if(!ARM_ENABLE_ACTUATION)control.stop("actuation_disabled");
 if(ARM_ENABLE_ACTUATION&&canReady&&(control.active||int32_t(stopUntil-now)>0)&&now-lastSend>=10){
  lastSend=now;twai_message_t out={};out.identifier=0x200;out.data_length_code=8;
  // IDs 1..3 always zero. Only ID4 has a bounded arm current.
  out.data[6]=uint16_t(control.output)>>8;out.data[7]=uint16_t(control.output)&255;
  if(twai_transmit(&out,0)==ESP_OK){++tx;submitted=true;}
  else{++failed;healthy=false;stop("tx_failed");}
 }
 static String report;static unsigned offset=0;
 if(now-lastReport>=1000&&report.isEmpty()){lastReport=now;report="STATUS "+status()+"\n";offset=0;}
 if(!report.isEmpty()){
  unsigned room=Serial.availableForWrite(),left=report.length()-offset,n=room<left?room:left;
  if(n)offset+=Serial.write(reinterpret_cast<const uint8_t *>(report.c_str()+offset),n);
  if(offset==report.length())report="";
 }
 delay(1);
}

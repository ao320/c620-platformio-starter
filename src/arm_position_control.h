#pragma once
#include "motor_control.h"
#include "position_control.h"
namespace arm_position {
// Empirically measured A->B distance; motor encoder counts, NOT joint degrees.
constexpr int32_t SPAN=56769, MARGIN=2838, JOG_COUNTS_PER_SECOND=2000, JOG_UP_COUNTS_PER_SECOND=20480;
constexpr int MAX_RPM=60, JOG_UP_MAX_RPM=150, MAX_DESCENT_RPM=15, OVERSPEED_RPM=1000, CURRENT_LIMIT=500;
constexpr uint16_t MANUAL_DEFAULT_CURRENT_LIMIT=3000;
constexpr int UP_CURRENT_RISE_PER_SECOND=6000;
constexpr int32_t JOG_UP_STEP_COUNTS=1000;
constexpr int32_t MANUAL_UPPER_HOLD=37931;
constexpr int DESCENT_CURRENT_LIMIT=3000;
constexpr int DESCENT_CURRENT_RISE_PER_SECOND=1000;
constexpr int UPWARD_BRAKE_CURRENT_LIMIT=3000;
constexpr int UPWARD_BRAKE_RISE_PER_SECOND=4000;
constexpr uint32_t JOG_UP_SETTLE_MS=200;
constexpr uint32_t LEASE_MS=250;
struct Control {
  bool homed=false,active=false,fault=false,travelHolding=false;
  // Opt-in: valid only when negative current lifts A->B against gravity.
  bool gravityAware=false,holdRequested=false;
  bool manualUpperHoldEnabled=true,manualUpperHold=false;
  int64_t origin=0;
  float target=0;
  uint32_t generation=0,session=0,started=0,heartbeat=0,lastStep=0;
  int direction=0;
  const int overspeedRpm=OVERSPEED_RPM;
  bool comparison=false,standalone=false;
  int64_t comparisonOrigin=0;
  uint16_t currentLimit=MANUAL_DEFAULT_CURRENT_LIMIT,manualLimit=MANUAL_DEFAULT_CURRENT_LIMIT;
  bool stallTiming=false;
  uint32_t stallSince=0;
  int16_t targetRpm=0,output=0;
  float rampedOutput=0,rampedRpm=0;
  // Active for the gravity-aware elbow controller.
  bool accelerationGuardEnabled=false,previousUpRpmValid=false;
  bool upStepMoving=false,upStepWaiting=false,upSettleTiming=false;
  uint32_t upSettleSince=0;
  int previousUpRpm=0;
  uint32_t accelerationGuardMs=0,accelerationEvents=0;
  void prepareSpeed(){
    // Current headroom must not multiply the feedback gains.
    speed.kpMilli=6000;speed.kiMilli=8000;
    speed.reset();rampedOutput=rampedRpm=0;holdRequested=false;manualUpperHold=false;
    previousUpRpmValid=false;previousUpRpm=0;accelerationGuardMs=0;accelerationEvents=0;
    upStepMoving=upStepWaiting=upSettleTiming=false;upSettleSince=0;
  }
  const char *reason="home_required";
  motor_control::SpeedPI speed;
  Control(){speed.kpMilli=6000;speed.kiMilli=8000;}
  int64_t position(const position_control::Encoder &e)const{return origin-e.counts;}
  void stop(const char *why){active=false;direction=0;targetRpm=output=0;speed.reset();rampedOutput=rampedRpm=0;holdRequested=false;manualUpperHold=false;travelHolding=false;accelerationGuardEnabled=false;previousUpRpmValid=false;accelerationGuardMs=0;stallTiming=false;reason=why;}
  int16_t gravityStep(int16_t targetSpeed,int measuredRpm,uint32_t dt,bool upperPositionOvershoot=false,bool descentRequested=false){
    // A->B lifting is negative current / negative RPM. Gravity acts positive.
    // No guessed constant feedforward: retain only the learned PI support term.
    const float error=float(targetSpeed)-measuredRpm;
    // Retain the integral as tentative support instead of removing it at
    // breakaway. The support current remains uncalibrated and load-dependent.
    const bool uphillOvershoot=measuredRpm< -5&&measuredRpm<targetSpeed;
    const bool downhillOvershoot=targetSpeed>=0&&measuredRpm>0&&measuredRpm>targetSpeed;
    const bool upwardBraking=measuredRpm< -5&&error>0;
    // Reduce the reverse-current correction during a commanded descent: the
    // previous gain could brake so hard that the arm appeared to reverse.
    const float kp=uphillOvershoot?36.0f:downhillOvershoot?18.0f:6.0f;
    const float previousIntegral=speed.integral;
    // Increase only the lifting startup accumulation; retain the existing
    // integral behavior while decelerating, holding, or descending.
    // If the upper hold target was passed, unwind upward integral faster,
    // but only when the speed error requests less lifting. Support remains
    // load-dependent and is not replaced by a guessed fixed current.
    const uint32_t integralGain=(descentRequested&&targetSpeed>0&&error>0)?75000:
                                (upperPositionOvershoot&&targetSpeed>0&&error>0)?100000:
                                (targetSpeed<0&&error<0)?40000:speed.kiMilli;
    float candidate=speed.integral+error*(integralGain*.000001f)*dt;
    const float positiveLimit=descentRequested?float(currentLimit<DESCENT_CURRENT_LIMIT?currentLimit:DESCENT_CURRENT_LIMIT):
      upwardBraking?float(currentLimit<UPWARD_BRAKE_CURRENT_LIMIT?currentLimit:UPWARD_BRAKE_CURRENT_LIMIT):0.0f;
    if(candidate>positiveLimit)candidate=positiveLimit;
    if(candidate< -float(currentLimit))candidate= -float(currentLimit);
    float raw=kp*error+candidate;
    if(!((raw< -float(currentLimit)&&error<0)||(raw>positiveLimit&&error>0)))speed.integral=candidate;
    else speed.integral=previousIntegral;
    raw=kp*error+speed.integral;
    if(raw>positiveLimit)raw=positiveLimit;
    if(raw< -float(currentLimit))raw= -float(currentLimit);
    // Bound normal torque changes in both directions. Safety stop still sets
    // zero immediately before this function is reached.
    // Allow lifting current to fall earlier when the motor exceeds the
    // upward target. Increasing support remains limited to the normal rate.
    const float reductionRate=(measuredRpm<0 && measuredRpm<targetSpeed-30)?8.0f:4.0f;
    // Positive current is normally reserved for commanded descent. Permit a
    // bounded opposing current only while measured motion is faster uphill
    // than requested, so the speed loop can actively brake instead of coasting.
    if(!descentRequested&&!upwardBraking&&rampedOutput>0)rampedOutput=0;
    if(raw>rampedOutput){
      if(rampedOutput<0){
        const float next=rampedOutput+reductionRate*dt;
        if(raw>next)raw=next;
        if(raw>0)raw=0;
      }else{
        const int rise=upwardBraking?UPWARD_BRAKE_RISE_PER_SECOND:DESCENT_CURRENT_RISE_PER_SECOND;
        if(raw>rampedOutput+rise*.001f*dt)raw=rampedOutput+rise*.001f*dt;
      }
    }
    const float negativeRise=(targetSpeed<0&&error<0)?UP_CURRENT_RISE_PER_SECOND:4000.0f;
    if(raw<rampedOutput-negativeRise*.001f*dt)raw=rampedOutput-negativeRise*.001f*dt;
    if(accelerationGuardEnabled){
      const int upRpm=measuredRpm<0?-measuredRpm:0;
      const int increase=upRpm-previousUpRpm;
      const bool accelerating=previousUpRpmValid&&measuredRpm<0&&increase>0&&
        uint64_t(increase)*1000>=uint64_t(dt)*3000;
      previousUpRpm=upRpm;previousUpRpmValid=true;
      if(measuredRpm>0)accelerationGuardMs=0; // Permit support on downward reversal.
      else if(accelerating){accelerationGuardMs=100;++accelerationEvents;}
      else if(dt>=accelerationGuardMs)accelerationGuardMs=0;
      else accelerationGuardMs-=dt;
      if(accelerationGuardMs&&raw<rampedOutput){
        raw=rampedOutput;
        if(error<0)speed.integral=previousIntegral; // No windup behind guard.
      }
      if(accelerating&&rampedOutput<0){
        if(error<0)speed.integral=previousIntegral;
        // Reduce at 2000 raw/s rather than a fixed amount per sample.
        // This keeps the response consistent if the loop interval changes.
        float reduced=rampedOutput+2.0f*dt;
        if(reduced>0)reduced=0;
        if(raw<reduced)raw=reduced;
      }
    }
    rampedOutput=raw;return int16_t(raw);
  }
  bool configureCurrent(uint16_t value,int rpm){
    if(active||rpm>5||rpm< -5||value<100||value>6000||value%100)return false;
    manualLimit=value;return true;
  }
  bool home(const position_control::Encoder &e,int rpm,uint32_t now){
    if(active||!e.initialized||now-e.lastMs>20||rpm>5||rpm< -5)return false;
    standalone=false;origin=e.counts;generation=e.generation;homed=true;fault=false;target=0;reason="homed_manual_A";return true;
  }
  bool enable(const position_control::Encoder &e,int rpm,uint32_t now,uint32_t id){
    const int64_t p=position(e);
    if(!homed||fault||active||!id||!e.initialized||e.generation!=generation||now-e.lastMs>20||
       rpm>5||rpm< -5||p<0||p>SPAN-MARGIN)return false;
    comparison=false;currentLimit=manualLimit;manualUpperHoldEnabled=true;
    accelerationGuardEnabled=gravityAware;
    session=id;active=true;started=heartbeat=lastStep=now;target=float(p);direction=0;
    prepareSpeed();stallTiming=false;reason="holding";return true;
  }
  bool startComparison(const position_control::Encoder &e,int rpm,uint32_t now,uint32_t id){
    const int64_t p=position(e);
    if(p+2000>SPAN-MARGIN)return false;
    if(!enable(e,rpm,now,id))return false;
    comparison=true;comparisonOrigin=p;currentLimit=manualLimit;target=float(p+2000);reason="comparison_running";return true;
  }
  bool startStandalone(const position_control::Encoder &e,int rpm,uint32_t now,uint32_t id){
    if(active||!id||!e.initialized||now-e.lastMs>20||rpm>5||rpm< -5)return false;
    homed=false;fault=false;standalone=true;comparison=true;origin=e.counts;generation=e.generation;
    comparisonOrigin=0;currentLimit=1000;target=2000;session=id;active=true;
    started=heartbeat=lastStep=now;direction=0;prepareSpeed();stallTiming=false;reason="standalone_running";return true;
  }
  bool command(uint32_t id,int d,uint32_t now){
    if(!active||id!=session||d< -1||d>1)return false;
    heartbeat=now;
    if(gravityAware&&!standalone&&!comparison&&direction!=0&&d==0)holdRequested=true;
    direction=comparison?0:d;return true;
  }
  void update(const position_control::Encoder &e,int rpm,uint32_t now,bool healthy){
    if((homed||(standalone&&active))&&(!e.initialized||e.generation!=generation||now-e.lastMs>50)){
      homed=false;fault=true;stop("reference_lost");
    }
    if(!active)return;
    const int64_t p=position(e);
    if(!healthy||(!homed&&!standalone)||now-e.lastMs>20){fault=true;stop("feedback_or_can_fault");return;}
    if(rpm>overspeedRpm||rpm< -overspeedRpm){fault=true;stop("overspeed");return;}
    // At the software travel edge, keep the session active and let the
    // position controller hold the nearest in-range target. Stopping here
    // removes gravity support and can let the arm fall away from the edge.
    if(!standalone&&(p<0||p>SPAN-MARGIN)){
      target=p<0?0.0f:float(SPAN-MARGIN);
      travelHolding=true;reason="travel_hold";
    }else if(travelHolding){travelHolding=false;reason="jogging";}
    if(now-heartbeat>LEASE_MS){stop("input_lost");return;}
    if(comparison&&p<comparisonOrigin-100){fault=true;stop("comparison_wrong_direction");return;}
    if(comparison&&p>=comparisonOrigin+2000){stop("comparison_travel_complete");return;}
    if(comparison&&now-started>=2000){stop("session_complete");return;}
    if(now-lastStep<10)return;
    const uint32_t dt=now-lastStep;lastStep=now;
    if(dt>50){fault=true;stop("control_timing_fault");return;}
    if(gravityAware&&!standalone&&holdRequested){
      // Capture the measured stop position; do not chase an old jog target.
      target=float(p);rampedRpm=0;holdRequested=false;
    }
    const bool manualProfile=gravityAware&&!standalone&&!comparison&&manualUpperHoldEnabled;
    if(manualProfile&&direction<0)manualUpperHold=false;
    if(manualProfile&&direction>0&&!manualUpperHold&&p>=MANUAL_UPPER_HOLD){
      manualUpperHold=true;target=float(p);rampedRpm=0;
    }
    const bool upperHolding=manualProfile&&manualUpperHold&&direction>=0;
    const bool fasterUpJog=gravityAware&&!standalone&&!comparison&&direction>0&&!upperHolding;
    if(fasterUpJog){
      if(!upStepMoving&&!upStepWaiting){
        target=float(p)+JOG_UP_STEP_COUNTS;upStepMoving=true;
      }else if(upStepMoving&&float(p)>=target){
        target=float(p);upStepMoving=false;upStepWaiting=true;upSettleTiming=false;
      }else if(upStepWaiting){
        if(rpm>=-10&&rpm<=10){
          if(!upSettleTiming){upSettleTiming=true;upSettleSince=now;}
          else if(now-upSettleSince>=JOG_UP_SETTLE_MS){
            target=float(p)+JOG_UP_STEP_COUNTS;upStepMoving=true;upStepWaiting=false;upSettleTiming=false;
          }
        }else upSettleTiming=false;
      }
    }else{
      upStepMoving=upStepWaiting=upSettleTiming=false;
      if(!upperHolding)target+=direction*(JOG_COUNTS_PER_SECOND*.001f)*dt;
    }
    if(!standalone){
      if(target<0)target=0;
      if(target>SPAN-MARGIN)target=SPAN-MARGIN;
    }
    // Position P produces motor RPM, followed by speed PI -> current request.
    float v=(target-float(p))*.04f;
    const int upLimit=fasterUpJog?JOG_UP_MAX_RPM:MAX_RPM;
    if(v>upLimit)v=upLimit;if(v< -MAX_RPM)v= -MAX_RPM;
    if(gravityAware&&!standalone&&v< -MAX_DESCENT_RPM)v= -MAX_DESCENT_RPM; // Gravity-assisted descent.
    float requestedRpm=-v; // Positive position points toward decreasing encoder B.
    // While a direction key is held, regulate a fixed motor-speed target.
    // Keep position control for release/hold and for soft travel boundaries.
    const bool velocityJog=gravityAware&&!standalone&&!comparison&&direction!=0&&!upperHolding;
    const bool outwardAtEdge=!standalone&&
      ((direction>0&&p>=SPAN-MARGIN)||(direction<0&&p<=0));
    if(velocityJog)requestedRpm=outwardAtEdge?0:(direction>0?-JOG_UP_MAX_RPM:MAX_DESCENT_RPM);
    if(currentLimit>2000||(gravityAware&&!standalone)){
      // Limit target acceleration; the gravity profile uses 30 RPM/s.
      if(rampedRpm*requestedRpm<0)rampedRpm=0;
      const float change=(fasterUpJog?.09f:gravityAware&&!standalone?.03f:.06f)*dt;
      if(requestedRpm>0&&requestedRpm>rampedRpm+change)requestedRpm=rampedRpm+change;
      if(requestedRpm<0&&requestedRpm<rampedRpm-change)requestedRpm=rampedRpm-change;
    }
    rampedRpm=requestedRpm;targetRpm=int16_t(requestedRpm);
    if(gravityAware&&!standalone){
      output=gravityStep(targetRpm,rpm,dt,upperHolding&&float(p)>target,direction<0&&!comparison&&!standalone&&manualUpperHoldEnabled);
    }else{
    const float previousIntegral=speed.integral;
    const int16_t desired=speed.step(targetRpm,rpm,currentLimit);
    if(currentLimit>2000){
      const float maxChange=4.0f*dt; // Limit torque buildup to 4000 raw/s.
      float next=desired;
      // Drop old-direction torque immediately; build reverse torque from zero
      // on the following step. Do not keep accelerating while PI asks to brake.
      if(rampedOutput*next<0)next=0;
      else {
        if(next>0&&next>rampedOutput+maxChange)next=rampedOutput+maxChange;
        if(next<0&&next<rampedOutput-maxChange)next=rampedOutput-maxChange;
      }
      const int error=int(targetRpm)-rpm;
      if((desired>next&&error>0)||(desired<next&&error<0))speed.integral=previousIntegral;
      rampedOutput=next;output=int16_t(next);
    }else{rampedOutput=desired;output=desired;}
    }
    const int absOutput=output<0?-int(output):int(output);
    const int absRpm=rpm<0?-rpm:rpm;
    const int absTarget=targetRpm<0?-int(targetRpm):int(targetRpm);
    if(absOutput>=currentLimit*9/10 && absRpm<3 && absTarget>=20){
      if(!stallTiming){stallTiming=true;stallSince=now;}
      else if(now-stallSince>=800){fault=true;stop("stalled_at_current_limit");return;}
    }else stallTiming=false;
    reason=standalone?"standalone_running":comparison?"comparison_running":upperHolding?"upper_hold":(direction?"jogging":"holding");
  }
};
}

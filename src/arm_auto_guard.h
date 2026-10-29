#pragma once
#include <stdint.h>

// Offline-tested guard for a future automatic trial. This header is not wired
// to motor control; position limits are trip points, not physical stops.
namespace arm_auto {
constexpr int64_t MIN_POSITION=15000, MAX_POSITION=35000;
constexpr int MAX_RPM=1000;
constexpr uint32_t LEASE_MS=250, LEG_MS=10000, TOTAL_MS=60000;
// Inner turnaround thresholds leave 2000 counts before the electrical trip.
// These margins do not guarantee a physical stopping distance.
constexpr int64_t LOWER_TURN=17000, UPPER_TURN=33000;
constexpr uint32_t SETTLE_MS=200;

struct Guard {
  bool active=false, latched=false;
  uint32_t started=0, legStarted=0, heartbeat=0;
  unsigned legs=0;
  int direction=0;
  bool settling=false,settleTiming=false;
  uint32_t settleSince=0;
  const char *reason="not_started";

  bool start(int64_t position, int rpm, bool homed, bool healthy, uint32_t now){
    if(active || latched || !homed || !healthy || position<MIN_POSITION ||
       position>MAX_POSITION || rpm< -5 || rpm>5) return false;
    active=true;started=legStarted=heartbeat=now;legs=1;direction=1;
    settling=settleTiming=false;reason="running";return true;
  }
  void fail(const char *why){active=false;latched=true;direction=0;reason=why;}
  void observe(int64_t position, int rpm, bool healthy, bool feedbackFresh, uint32_t now){
    if(!active)return;
    if(!healthy || !feedbackFresh){fail("feedback_or_can_fault");return;}
    if(position<MIN_POSITION || position>MAX_POSITION){fail("range_exceeded");return;}
    if(rpm< -MAX_RPM || rpm>MAX_RPM){fail("overspeed");return;}
    if(uint32_t(now-heartbeat)>LEASE_MS){fail("supervisor_lost");return;}
    if(uint32_t(now-started)>=TOTAL_MS){fail("overall_timeout");return;}
    if(uint32_t(now-legStarted)>=LEG_MS){fail("leg_timeout");return;}
  }
  void renew(uint32_t now){
    if(!active)return;
    if(uint32_t(now-heartbeat)>LEASE_MS){fail("supervisor_lost");return;}
    heartbeat=now;
  }
  // Called at control-loop frequency; caller must not use the 1 Hz UI for
  // these decisions. Zero direction requests position hold, not current zero.
  void update(int64_t position, int rpm, bool healthy, bool feedbackFresh, uint32_t now){
    observe(position,rpm,healthy,feedbackFresh,now);
    if(!active)return;
    if(!settling && ((direction>0 && position>=UPPER_TURN) ||
                     (direction<0 && position<=LOWER_TURN))){
      settling=true;settleTiming=false;
    }
    if(!settling)return;
    if(rpm< -10 || rpm>10){settleTiming=false;return;}
    if(!settleTiming){settleTiming=true;settleSince=now;return;}
    if(uint32_t(now-settleSince)<SETTLE_MS)return;
    if(legs>=6){active=false;direction=0;reason="complete";return;}
    ++legs;legStarted=now;direction= -direction;settling=settleTiming=false;
  }
  int requestedDirection()const{return active&&!settling?direction:0;}
};
}

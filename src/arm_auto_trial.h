#pragma once
#include "arm_auto_guard.h"
#include "arm_position_control.h"

// Automatic supervisor adapter. It has no serial or CAN access. Callers must
// drive tick() from the ESP loop and preserve the independent external lease.
namespace arm_auto {
struct Trial {
  Guard guard;
  uint32_t session=0;
  bool start(arm_position::Control &c, const position_control::Encoder &e,
             int rpm, bool healthy, uint32_t now, uint32_t id){
    if(!id || c.manualLimit!=6000 || !c.gravityAware || c.active || c.fault ||
       !e.initialized || e.generation!=c.generation || uint32_t(now-e.lastMs)>20)
      return false;
    if(!guard.start(c.position(e),rpm,c.homed,healthy,now))return false;
    if(!c.enable(e,rpm,now,id)){guard.fail("enable_rejected");return false;}
    c.manualUpperHoldEnabled=false; // Automatic trial has its own range guard.
    session=id;return true;
  }
  void renew(uint32_t id,uint32_t now){if(id==session)guard.renew(now);}
  void tick(arm_position::Control &c,const position_control::Encoder &e,
            int rpm,bool healthy,uint32_t now){
    if(!guard.active)return;
    if(!c.active || c.session!=session){guard.fail("controller_stopped");return;}
    const unsigned previousLeg=guard.legs;
    const bool fresh=e.initialized&&e.generation==c.generation&&uint32_t(now-e.lastMs)<=20;
    guard.update(c.position(e),rpm,healthy,fresh,now);
    if(!guard.active){c.stop(guard.reason);return;}
    // Existing 10 s timer is per leg; the guard also has a total deadline.
    if(guard.legs!=previousLeg){
      c.started=now;
      // The arm may have drifted while settling. Begin descent from the
      // current measured position rather than chasing the old hold target.
      if(guard.direction<0){c.target=float(c.position(e));c.rampedRpm=0;}
    }
    if(!c.command(session,guard.requestedDirection(),now)){
      guard.fail("command_rejected");c.stop(guard.reason);
    }
  }
};
}

#pragma once
#include <stdint.h>
#include <string.h>

// Explicit, one-shot diagnostic; never started by boot or an HTTP request.
namespace startup_calibration {
enum class Stage { Idle, Ramp, Settle, Done, Aborted };
struct Result {
  bool tested=false, detected=false;
  int16_t command=0, rpm=0;
  uint32_t elapsedMs=0;
};
class Calibration {
 public:
  Stage stage=Stage::Idle;
  Result results[6];
  unsigned trial=0;
  int16_t output=0, submitted=0;
  const char *reason="not_started";
  bool active() const { return stage==Stage::Ramp || stage==Stage::Settle; }
  bool start(uint32_t now, bool ready) {
    if (stage!=Stage::Idle || !ready) return false;
    stage=Stage::Ramp;reason="running";started=nextSend=now;return true;
  }
  void abort(uint32_t now, const char *why) {
    if (!active()) return;
    failed=true;reason=why;stage=Stage::Settle;started=now;still=false;output=0;nextSend=now;
  }
  bool frame(uint32_t now, const int16_t rpm[3], bool fresh, bool healthy, bool stop, uint8_t data[8]) {
    if (!active()) return false;
    if (!failed && (stop || !healthy || !fresh))
      abort(now,stop ? "user_stop" : (!healthy ? "can_fault" : "feedback_lost"));
    bool allStopped=true;
    for(unsigned i=0;i<3;++i) allStopped &= rpm[i]==0;
    if(stage==Stage::Ramp) {
      const int value=rpm[trial/2];
      const bool detected=value>=20 || value<=-20;
      if(detected || uint32_t(now-started)>=1000) {
        Result &r=results[trial];r.tested=true;r.detected=detected;
        r.command=submitted;r.rpm=value;r.elapsedMs=now-started;
        stage=Stage::Settle;started=now;stillSince=now;still=false;output=0;nextSend=now;
      } else {
        const int magnitude=now-started;
        output=(trial%2) ? -magnitude : magnitude;
      }
    }
    if(stage==Stage::Settle) {
      output=0;
      if(!allStopped || !fresh) still=false;
      else if(!still) {still=true;stillSince=now;}
      if(uint32_t(now-started)>=1000 && still && uint32_t(now-stillSince)>=200) {
        if(failed) stage=Stage::Aborted;
        else if(++trial==6) {stage=Stage::Done;reason="complete";}
        else {stage=Stage::Ramp;started=now;submitted=0;}
      } else if(uint32_t(now-started)>=5000) {
        stage=Stage::Aborted;reason="stop_not_confirmed";
      }
    }
    // Always submit a final zero when leaving the sequence.
    if(!active()) output=0;
    if(int32_t(now-nextSend)<0) return false;
    nextSend=now+10;
    memset(data,0,8);
    if(stage==Stage::Ramp && output) {
      const unsigned index=trial/2*2;
      data[index]=uint16_t(output)>>8;data[index+1]=uint16_t(output)&255;
    }
    return true;
  }
  void accepted() {submitted=output;}
 private:
  uint32_t started=0,nextSend=0,stillSince=0;
  bool failed=false,still=false;
};
inline const char *name(Stage stage) {
  switch(stage) {
    case Stage::Idle:return "idle";case Stage::Ramp:return "ramp";
    case Stage::Settle:return "settle";case Stage::Done:return "done";
    case Stage::Aborted:return "aborted";
  }
  return "unknown";
}
}

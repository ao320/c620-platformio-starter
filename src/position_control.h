#pragma once
#include <stdint.h>
namespace position_control {
constexpr int64_t COUNTS_NUM=8192LL*3591;
constexpr int64_t COUNTS_DEN=187LL*36000;
inline int64_t countsForCdeg(int32_t cdeg) {
  const int64_t n=int64_t(cdeg)*COUNTS_NUM;
  return (n+(n<0?-COUNTS_DEN/2:COUNTS_DEN/2))/COUNTS_DEN;
}
inline int32_t cdegForCounts(int64_t counts) {
  const int64_t n=counts*COUNTS_DEN;
  return int32_t((n+(n<0?-COUNTS_NUM/2:COUNTS_NUM/2))/COUNTS_NUM);
}
struct Encoder {
  bool initialized=false;
  uint16_t previous=0;
  uint32_t lastMs=0,generation=0;
  int64_t counts=0;
  void observe(uint16_t raw,uint32_t now) {
    if(raw>=8192) {initialized=false;++generation;return;}
    if(initialized && now-lastMs<=20) {
      int delta=int(raw)-previous;
      if(delta==4096 || delta==-4096) ++generation;
      else {if(delta>4096)delta-=8192;if(delta< -4096)delta+=8192;counts+=delta;}
    } else if(initialized) ++generation;
    initialized=true;previous=raw;lastMs=now;
  }
};
struct Move {
  bool moving=false,settling=false;
  const char *reason="idle";
  int32_t requestedCdeg=0,actualCdeg=0,errorCdeg=0;
  int16_t targetRpm=0;
  int64_t origin=0,target=0;
  uint32_t started=0,settledSince=0,generation=0;
  bool start(int32_t cdeg,const Encoder &encoder,uint32_t now) {
    if(moving || !encoder.initialized || now-encoder.lastMs>20 || !cdeg || cdeg< -18000 || cdeg>18000)return false;
    moving=true;settling=false;reason="moving";requestedCdeg=cdeg;
    actualCdeg=0;errorCdeg=cdeg;targetRpm=0;origin=encoder.counts;
    target=origin+countsForCdeg(cdeg);generation=encoder.generation;started=now;return true;
  }
  void abort(const char *why) {moving=false;targetRpm=0;settling=false;reason=why;}
  void update(const Encoder &encoder,int16_t rpm,uint32_t now) {
    if(requestedCdeg && encoder.initialized && encoder.generation==generation) {
      actualCdeg=cdegForCounts(encoder.counts-origin);
      errorCdeg=cdegForCounts(target-encoder.counts);
    }
    if(!moving)return;
    if(!encoder.initialized || now-encoder.lastMs>20 || encoder.generation!=generation) {abort("angle_feedback_lost");return;}
    if((requestedCdeg>0 && actualCdeg< -1000) || (requestedCdeg<0 && actualCdeg>1000) ||
        actualCdeg> (requestedCdeg>0?requestedCdeg:0)+3000 || actualCdeg< (requestedCdeg<0?requestedCdeg:0)-3000) {
      abort("position_travel_limit");return;
    }
    if(rpm>400 || rpm< -400) {abort("position_overspeed");return;}
    if(now-started>=6000) {abort("position_timeout");return;}
    const int error=errorCdeg<0?-errorCdeg:errorCdeg;
    if(error<=100) {
      targetRpm=0;
      if(rpm>=-20 && rpm<=20) {
        if(!settling) {settling=true;settledSince=now;}
        if(now-settledSince>=200) {abort("complete");return;}
      } else settling=false;
    } else {
      settling=false;
      int velocity=errorCdeg/5; // 20 motor RPM per wheel degree.
      if(velocity>200)velocity=200;if(velocity< -200)velocity=-200;
      if(velocity>0 && velocity<60)velocity=60;
      if(velocity<0 && velocity> -60)velocity=-60;
      targetRpm=int16_t(velocity);
    }
  }
};
}

#pragma once
#include <stdint.h>
namespace arm_trace {
// Bounded RAM recorder. Serialization happens only in the 1 Hz status report.
struct Sample {
  uint32_t ms=0;
  int64_t position=0;
  int32_t target=0;
  int16_t targetRpm=0,rpm=0,command=0,current=0;
  float integral=0;
  uint32_t feedbackAgeMs=0;
};
struct Recorder {
  // 10 seconds at 10 ms, plus stop snapshots and post-stop samples.
  // Export remains limited to eight samples per 1 Hz status report.
  static constexpr unsigned CAPACITY=1024, POST_SAMPLES=16, CHUNK=8;
  Sample samples[CAPACITY];
  unsigned head=0,count=0,post=0,exported=0;
  uint32_t periodMs=10;
  uint32_t session=0,last=0;
  bool recording=false,stopping=false,ready=false;
  const char *reason="none";
  void begin(uint32_t id,uint32_t intervalMs=10){
    periodMs=intervalMs?intervalMs:10;
    head=count=post=exported=0;session=id;last=0;
    recording=true;stopping=ready=false;reason="recording";
  }
  void append(const Sample &s){samples[head]=s;head=(head+1)%CAPACITY;if(count<CAPACITY)++count;last=s.ms;}
  void observe(const Sample &s){
    if(!recording||uint32_t(s.ms-last)<periodMs)return;
    append(s);
    if(stopping&&++post>=POST_SAMPLES){recording=false;ready=true;}
  }
  void stop(const Sample &before,const Sample &after,const char *why){
    if(!recording||stopping)return;
    append(before);append(after);stopping=true;post=0;reason=why;
  }
  const Sample &at(unsigned i)const{return samples[(head+CAPACITY-count+i)%CAPACITY];}
  // Continue cycling until the next session, allowing missed chunks to be
  // recovered from subsequent status reports.
  void advanceExport(unsigned end){exported=(ready&&end>=count)?0:end;}
};
}

#pragma once
#include <stdint.h>
struct UdpGuard {
  uint32_t run=0, seq=0, peer=0;
  static bool fresh(uint32_t expires,uint32_t now) {
    const int32_t remaining=static_cast<int32_t>(expires-now);
    return remaining>0 && remaining<=150;
  }
  bool velocity(uint32_t id,uint32_t sequence,uint32_t expires,uint32_t now,
                uint32_t source,bool neutral,bool ready,uint32_t current,bool running) {
    if(!id || !sequence || !fresh(expires,now))return false;
    if(ready && id==current)return false;
    if(id!=run) {
      if(!ready || !neutral || id==current)return false;
      run=id;seq=0;peer=source;
    }
    if(source!=peer || sequence<=seq || (!(ready && neutral) && !(running && current==id)))return false;
    seq=sequence;return true;
  }
  bool stop(uint32_t id,uint32_t sequence,uint32_t expires,uint32_t now,uint32_t source,uint32_t current) {
    if(!id || id!=run || id!=current || source!=peer || !sequence || sequence<=seq || !fresh(expires,now))return false;
    seq=sequence;return true;
  }
};

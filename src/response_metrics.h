#pragma once
#include <stdint.h>

// Observation only: one sample per 10 ms control frame. Freeze at zero/stop.
// A target change begins a new step; three consecutive samples confirm motion.
struct ResponseMetrics {
  int16_t target = 0, window[10] = {};
  uint32_t started = 0, samples = 0, absoluteError = 0;
  int32_t motionMs = -1, reach90Ms = -1, sum = 0;
  uint16_t peakMean100 = 0;
  uint8_t index = 0, count = 0, moving = 0, reaching = 0;
  void reset() { *this = ResponseMetrics{}; }
  void observe(int16_t requested, int16_t rpm, uint32_t now) {
    if (!requested) return;
    if (requested != target) { reset(); target = requested; started = now; }
    const int aligned = target > 0 ? int(rpm) : -int(rpm);
    const int magnitude = target > 0 ? int(target) : -int(target);
    const int error = int(target) - int(rpm);
    absoluteError += error < 0 ? -error : error;
    ++samples;
    moving = aligned >= 20 ? (moving < 3 ? moving + 1 : 3) : 0;
    reaching = aligned * 10 >= magnitude * 9 ? (reaching < 3 ? reaching + 1 : 3) : 0;
    // Report confirmation time, not an inferred timestamp before confirmation.
    if (motionMs < 0 && moving == 3) motionMs = now - started;
    if (reach90Ms < 0 && reaching == 3) reach90Ms = now - started;
    sum -= window[index]; window[index] = rpm; sum += rpm;
    index = (index + 1) % 10;
    if (count < 10) ++count;
    if (count == 10) {
      const unsigned mean = (sum < 0 ? -sum : sum) / 10;
      if (mean > peakMean100) peakMean100 = mean;
    }
  }
};

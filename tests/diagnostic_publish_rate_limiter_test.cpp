#include <cassert>
#include <cstdint>

#include "../components/esp_wifi_sensing/diagnostic_publish_rate_limiter.h"
#include "../components/esp_wifi_sensing/adaptive_motion_detector.h"

using esphome::esp_wifi_sensing::AdaptiveMotionDetector;
using esphome::esp_wifi_sensing::DiagnosticPublishRateLimiter;

int main() {
  {
    DiagnosticPublishRateLimiter limiter;
    assert(limiter.should_publish(0));
    assert(!limiter.should_publish(1));
    assert(!limiter.should_publish(999));
    assert(limiter.should_publish(1000));
    assert(!limiter.should_publish(1500));
    assert(limiter.should_publish(2000));
  }

  {
    DiagnosticPublishRateLimiter limiter;
    AdaptiveMotionDetector detector;
    detector.set_warmup_time_ms(0);
    detector.set_learning_delay_ms(0);
    detector.set_baseline_rise_time_ms(1000);
    detector.set_baseline_fall_time_ms(1000);

    auto first = detector.update(1000, 0);
    assert(limiter.should_publish(0));
    auto latest = first;

    for (uint32_t now = 100; now < 1000; now += 100) {
      latest = detector.update(1000 + now, now);
      assert(!limiter.should_publish(now));
    }

    auto at_one_second = detector.update(2500, 1000);
    assert(limiter.should_publish(1000));
    assert(at_one_second.baseline_mean == detector.baseline_mean());
    assert(at_one_second.baseline_mean != latest.baseline_mean);
  }

  return 0;
}

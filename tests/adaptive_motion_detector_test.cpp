#include <cassert>
#include <cmath>
#include <cstdint>

#include "../components/esp_wifi_sensing/adaptive_motion_detector.h"

using esphome::esp_wifi_sensing::AdaptiveMotionDetector;
using esphome::esp_wifi_sensing::AdaptiveMotionDetectorResult;

static bool near(float actual, float expected) { return std::fabs(actual - expected) < 0.001f; }

static AdaptiveMotionDetector make_detector() {
  AdaptiveMotionDetector detector;
  detector.set_warmup_time_ms(0);
  detector.set_learning_delay_ms(0);
  detector.set_baseline_rise_time_ms(0);
  detector.set_baseline_fall_time_ms(0);
  detector.set_motion_hold_time_ms(300);
  detector.set_debounce_samples(2);
  detector.set_sigma_multiplier(3.0f);
  return detector;
}

int main() {
  {
    AdaptiveMotionDetector detector = make_detector();
    auto first = detector.update(1000, 0);
    assert(!first.motion);
    assert(!detector.persistence_has_valid_window());
  }

  {
    AdaptiveMotionDetector detector = make_detector();
    detector.set_warmup_time_ms(1000);
    detector.update(1000, 0);
    auto warmup = detector.update(5000, 500);
    assert(!warmup.motion);
    assert(!warmup.persistence_on);
  }

  {
    AdaptiveMotionDetector detector = make_detector();
    detector.set_baseline_rise_time_ms(1000000);
    detector.set_baseline_fall_time_ms(1000000);
    detector.update(1000, 0);
    auto one = detector.update(2000, 100);
    assert(one.candidate);
    assert(!one.persistence_on);
    assert(!one.motion);
    for (uint8_t i = 0; i < 9; i++) {
      auto sample = detector.update(1000, 200 + i * 100);
      assert(!sample.motion);
    }
  }

  {
    AdaptiveMotionDetector detector = make_detector();
    detector.set_baseline_rise_time_ms(1000000);
    detector.set_baseline_fall_time_ms(1000000);
    detector.update(1000, 0);
    auto one = detector.update(2000, 100);
    auto two = detector.update(2000, 200);
    assert(one.candidate);
    assert(two.candidate);
    assert(!two.persistence_on);
    assert(!two.motion);
  }

  {
    AdaptiveMotionDetector detector = make_detector();
    detector.set_baseline_rise_time_ms(1000000);
    detector.set_baseline_fall_time_ms(1000000);
    detector.update(1000, 0);
    bool motion = false;
    bool persistence = false;
    for (uint8_t i = 0; i < 11; i++) {
      auto result = detector.update(2000, 100 + i * 100);
      motion = result.motion;
      persistence = result.persistence_on;
    }
    assert(persistence);
    assert(motion);
  }

  {
    AdaptiveMotionDetector detector = make_detector();
    detector.set_baseline_rise_time_ms(1000000);
    detector.set_baseline_fall_time_ms(1000000);
    detector.update(1000, 0);
    for (uint8_t i = 0; i < 10; i++) {
      auto result = detector.update(i < 7 ? 2000 : 1000, 100 + i * 100);
      if (i < 9) {
        assert(!result.persistence_on);
      } else {
        assert(result.persistence_on);
      }
    }
  }

  {
    AdaptiveMotionDetector detector = make_detector();
    detector.set_baseline_rise_time_ms(1000000);
    detector.set_baseline_fall_time_ms(1000000);
    detector.update(1000, 0);
    for (uint8_t i = 0; i < 11; i++) {
      detector.update(2000, 100 + i * 100);
    }
    assert(detector.motion());
    auto held = detector.update(1000, 1200);
    assert(held.motion);
    AdaptiveMotionDetectorResult released{};
    for (uint8_t i = 0; i < 4; i++) {
      released = detector.update(1000, 1300 + i * 100);
    }
    assert(!released.persistence_on);
    assert(!released.motion);
  }

  {
    AdaptiveMotionDetector detector = make_detector();
    detector.set_baseline_rise_time_ms(1000000);
    detector.set_baseline_fall_time_ms(1000000);
    detector.update(1000, 0);
    detector.update(1000000, 100);
    for (uint8_t i = 0; i < 9; i++) {
      auto result = detector.update(1000, 200 + i * 100);
      assert(!result.motion);
    }
    assert(!detector.persistence_on());
  }

  {
    AdaptiveMotionDetector detector = make_detector();
    detector.set_baseline_rise_time_ms(1000);
    detector.set_baseline_fall_time_ms(1000);
    detector.update(1000, 0);
    auto result = detector.update(1100, 100);
    assert(result.baseline_mean > 1000.0f);
  }

  {
    AdaptiveMotionDetector detector = make_detector();
    detector.update(1000, 0);
    auto quiet = detector.update(1000, 100);
    assert(!quiet.candidate);
    assert(!quiet.motion);
  }

  return 0;
}

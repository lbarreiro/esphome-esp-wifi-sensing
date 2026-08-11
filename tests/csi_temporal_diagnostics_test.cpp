#include <cassert>
#include <cmath>
#include <cstdint>

#include "../components/esp_wifi_sensing/csi_temporal_diagnostics.h"

using esphome::esp_wifi_sensing::CsiTemporalDiagnostics;

static bool near(float actual, float expected) {
  return std::fabs(actual - expected) < 0.001f;
}

int main() {
  {
    CsiTemporalDiagnostics diagnostics;
    auto first = diagnostics.update(1000);
    assert(!first.has_delta);
    auto second = diagnostics.update(1000);
    assert(second.has_delta);
    assert(second.delta == 0);
    assert(near(second.temporal_mean, 0.0f));
    diagnostics.update(1000);
    auto fourth = diagnostics.update(1000);
    assert(fourth.delta == 0);
    assert(near(fourth.temporal_mean, 0.0f));
  }

  {
    CsiTemporalDiagnostics diagnostics;
    assert(!diagnostics.update(1000).has_delta);
    auto step = diagnostics.update(1100);
    assert(step.has_delta);
    assert(step.delta == 100);
    auto steady = diagnostics.update(1100);
    assert(steady.delta == 0);
    auto steady_again = diagnostics.update(1100);
    assert(steady_again.delta == 0);
  }

  {
    CsiTemporalDiagnostics diagnostics;
    assert(!diagnostics.update(1000).has_delta);
    assert(diagnostics.update(1200).delta == 200);
    assert(diagnostics.update(1000).delta == 200);
    assert(diagnostics.update(1200).delta == 200);
    assert(diagnostics.update(1000).delta == 200);
  }

  {
    CsiTemporalDiagnostics diagnostics;
    assert(!diagnostics.update(0).has_delta);
    for (uint32_t metric = 1; metric <= CsiTemporalDiagnostics::kWindowSize; metric++) {
      diagnostics.update(metric);
    }
    auto after_full_window = diagnostics.update(20);
    assert(after_full_window.delta == 10);
    assert(near(after_full_window.temporal_mean, 1.9f));
  }

  {
    CsiTemporalDiagnostics diagnostics;
    assert(!diagnostics.update(1000).has_delta);
    diagnostics.update(1100);  // delta 100, mean 100, persistence 0%
    auto sample = diagnostics.update(1100);  // delta 0, mean 50, persistence 50%
    assert(sample.delta == 0);
    assert(near(sample.temporal_mean, 50.0f));
    assert(near(sample.temporal_persistence, 50.0f));
  }

  return 0;
}

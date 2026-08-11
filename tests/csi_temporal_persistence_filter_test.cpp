#include <cassert>
#include <cmath>
#include <cstdint>

#include "../components/esp_wifi_sensing/csi_temporal_persistence_filter.h"

using esphome::esp_wifi_sensing::CsiTemporalPersistenceFilter;

static bool near(float actual, float expected) {
  return std::fabs(actual - expected) < 0.001f;
}

static bool feed_delta_window(CsiTemporalPersistenceFilter &filter, const uint32_t *deltas, uint8_t count) {
  bool state = false;
  for (uint8_t i = 0; i < count; i++) {
    state = filter.update_delta(deltas[i]);
  }
  return state;
}

int main() {
  {
    CsiTemporalPersistenceFilter filter;
    const uint32_t deltas[] = {0, 0, 0, 0, 1000, 0, 0, 0, 0, 0};
    const bool state = feed_delta_window(filter, deltas, 10);
    assert(!state);
    assert(near(filter.persistence(), 0.1f));
  }

  {
    CsiTemporalPersistenceFilter filter;
    const uint32_t deltas[] = {1000, 0, 0, 1000, 0, 0, 0, 0, 0, 0};
    const bool state = feed_delta_window(filter, deltas, 10);
    assert(!state);
    assert(near(filter.persistence(), 0.2f));
  }

  {
    CsiTemporalPersistenceFilter filter;
    const uint32_t deltas[] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 0, 0, 0};
    const bool state = feed_delta_window(filter, deltas, 10);
    assert(state);
    assert(near(filter.persistence(), 0.7f));
  }

  {
    CsiTemporalPersistenceFilter filter;
    const uint32_t deltas[] = {1, 1, 1, 1, 1, 1, 1, 0, 0, 0};
    assert(feed_delta_window(filter, deltas, 10));
    assert(filter.is_on());
  }

  {
    CsiTemporalPersistenceFilter filter;
    const uint32_t deltas[] = {1, 1, 1, 1, 1, 1, 0, 0, 0, 0};
    assert(!feed_delta_window(filter, deltas, 10));
    assert(!filter.is_on());
  }

  {
    CsiTemporalPersistenceFilter filter;
    const uint32_t start[] = {1, 1, 1, 1, 1, 1, 1, 0, 0, 0};
    assert(feed_delta_window(filter, start, 10));
    assert(!filter.update_delta(0));
    assert(near(filter.persistence(), 0.6f));
  }

  {
    CsiTemporalPersistenceFilter filter;
    const uint32_t deltas[] = {1000000, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    assert(!feed_delta_window(filter, deltas, 10));
    assert(near(filter.persistence(), 0.1f));
  }

  {
    CsiTemporalPersistenceFilter filter;
    for (uint8_t i = 0; i < CsiTemporalPersistenceFilter::kWindowSize - 1; i++) {
      assert(!filter.update_delta(1));
    }
    assert(!filter.has_valid_window());
    assert(!filter.is_on());
  }

  {
    CsiTemporalPersistenceFilter filter;
    const uint32_t deltas[] = {0, 3, 0, 0, 5, 0, 0, 0, 0, 0};
    assert(!feed_delta_window(filter, deltas, 10));
    assert(near(filter.persistence(), 0.2f));
  }

  {
    CsiTemporalPersistenceFilter filter;
    assert(!filter.update_metric(1000));
    assert(!filter.has_valid_window());
  }

  return 0;
}

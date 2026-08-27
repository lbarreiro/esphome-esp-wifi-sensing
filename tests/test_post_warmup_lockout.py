from pathlib import Path

POST_WARMUP_LOCKOUT_MS = 60000


class MotionBinarySensor:
    def __init__(self):
        self.states = []

    def publish_state(self, state):
        self.states.append(state)


class LockoutSimulator:
    def __init__(self):
        self.warmup_complete_logged = False
        self.post_warmup_lockout_complete_logged = False
        self.post_warmup_lockout_start_ms = 0
        self.motion_state = True
        self.consecutive_above_threshold = 4
        self.last_motion_time = 1234
        self.motion_binary_sensor = MotionBinarySensor()
        self.events = []

    def clear_motion_candidate_state(self):
        self.consecutive_above_threshold = 0
        self.motion_state = False
        self.last_motion_time = 0
        self.events.append("clear")

    def force_motion_off(self):
        self.clear_motion_candidate_state()
        self.motion_binary_sensor.publish_state(False)
        self.events.append("publish_off")

    def warmup_complete(self, now):
        self.events.append("warmup_complete_log")
        self.force_motion_off()
        self.events.append("lockout_start_log")
        self.post_warmup_lockout_start_ms = now
        self.events.append("lockout_start")
        self.warmup_complete_logged = True

    def post_warmup_lockout_active(self, now):
        if not self.warmup_complete_logged:
            return False
        if now - self.post_warmup_lockout_start_ms < POST_WARMUP_LOCKOUT_MS:
            return True
        if not self.post_warmup_lockout_complete_logged:
            self.force_motion_off()
            self.events.append("lockout_complete_log")
            self.post_warmup_lockout_complete_logged = True
        return False

    def lockout_loop(self, now, above_threshold=False):
        if self.post_warmup_lockout_active(now):
            self.force_motion_off()
            self.events.append("baseline_update_motion_false")
            return False
        if above_threshold:
            self.consecutive_above_threshold += 1
            self.motion_state = self.consecutive_above_threshold >= 2
            self.motion_binary_sensor.publish_state(self.motion_state)
            return self.motion_state
        self.consecutive_above_threshold = 0
        self.motion_state = False
        self.motion_binary_sensor.publish_state(False)
        return False


def test_warmup_completion_forces_off_clears_then_starts_lockout():
    sim = LockoutSimulator()

    sim.warmup_complete(120000)

    assert sim.motion_state is False
    assert sim.motion_binary_sensor.states[-1] is False
    assert sim.consecutive_above_threshold == 0
    assert sim.last_motion_time == 0
    assert sim.post_warmup_lockout_active(120000) is True
    assert sim.events[:5] == [
        "warmup_complete_log",
        "clear",
        "publish_off",
        "lockout_start_log",
        "lockout_start",
    ]


def test_motion_remains_off_during_lockout_when_starting_on():
    sim = LockoutSimulator()
    sim.warmup_complete(120000)

    sim.motion_state = True
    sim.consecutive_above_threshold = 9
    sim.last_motion_time = 150000
    sim.lockout_loop(150000)

    assert sim.motion_state is False
    assert sim.motion_binary_sensor.states[-1] is False
    assert sim.consecutive_above_threshold == 0
    assert sim.last_motion_time == 0


def test_threshold_crossings_during_lockout_cannot_publish_motion_on():
    sim = LockoutSimulator()
    sim.warmup_complete(120000)

    for now in (121000, 122000, 150000, 179999):
        assert sim.lockout_loop(now, above_threshold=True) is False
        assert sim.motion_state is False
        assert sim.motion_binary_sensor.states[-1] is False
        assert sim.consecutive_above_threshold == 0


def test_lockout_expiry_starts_from_clean_off_state():
    sim = LockoutSimulator()
    sim.warmup_complete(120000)
    sim.motion_state = True
    sim.consecutive_above_threshold = 6
    sim.last_motion_time = 179000

    assert sim.post_warmup_lockout_active(180000) is False

    assert sim.motion_state is False
    assert sim.motion_binary_sensor.states[-1] is False
    assert sim.consecutive_above_threshold == 0
    assert sim.last_motion_time == 0
    assert "lockout_complete_log" in sim.events


def test_motion_after_lockout_uses_existing_debounce_pipeline():
    sim = LockoutSimulator()
    sim.warmup_complete(120000)
    assert sim.post_warmup_lockout_active(180000) is False

    assert sim.lockout_loop(181000, above_threshold=True) is False
    assert sim.lockout_loop(182000, above_threshold=True) is True

    assert sim.motion_state is True
    assert sim.motion_binary_sensor.states[-1] is True


def test_source_orders_force_off_before_lockout_start():
    source = Path("components/esp_wifi_sensing/wifi_sensing.cpp").read_text()
    warmup_block = source.split('if (this->warmup_time_ms_ > 0 && !this->warmup_complete_logged_) {', 1)[1].split('}', 1)[0]

    assert warmup_block.index('ESP_LOGI(TAG, "Warm-up complete - forcing motion OFF")') < warmup_block.index('this->force_motion_off_();')
    assert warmup_block.index('this->force_motion_off_();') < warmup_block.index('ESP_LOGI(TAG, "Motion state cleared - starting 60s motion lockout")')
    assert warmup_block.index('ESP_LOGI(TAG, "Motion state cleared - starting 60s motion lockout")') < warmup_block.index('this->post_warmup_lockout_start_ms_ = now;')

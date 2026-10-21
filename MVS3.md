# Three independent CSI branches

Base: `feature/mvs-configurable-parameters`, commit
`7a3b693e1d8edbe6288f967c7e5607c3ee524828`.
Work branch: `feature/mvs3-raw-episode-learning`.

## Data flow

One existing CSI receiver and one existing ping session feed all branches.
No additional traffic is generated and the ping interval is unchanged.

1. **Reference MVS-256**: existing pipeline, parser, MVS, calibration, filter,
   hold and binary sensor. `mvs_algorithm.h/.cpp` are unchanged. Neither its
   observations nor its decisions are inputs to the learner.
2. **Comparison MVS-128**: existing separate parser/MVS and binary sensor,
   unchanged. It cannot influence either 256 branch.
3. **Learned MVS-256 (MVS-3)**: independent parser of the original 256-byte
   packet (before gain compensation), independent observation accumulator,
   jitter calibration, confirmation filter and hold. Only this branch uses the
   learner. It is enabled by configuring `motion_3`.

All three use the existing configured MVS parameters. The experimental MVS is
kept in separate files so its changes cannot alter either reference algorithm.

## Learning, step by step

The 48 positions are the existing MVS bins, not 48 Wi-Fi channels. They aggregate
the parser's valid subcarriers into log-amplitude observations. All 48 always
contribute. Common-mode removal and temporal differences happen **before any
weighting**. The learner receives the square of each unweighted difference.

* **Silence:** contiguous 5-second blocks, at least 20 observations per block,
  with low raw energy and no experimental held ON state. Ambiguous observations
  discard the partial block. Six complete blocks bootstrap the independent raw
  gate. Per-bin quiet energy is averaged with a capped 24-block time scale.
* **Movement response:** a raw episode requires at least six responsive bins
  and a raw energy increase relative to the quiet profile. It ends after two
  seconds of raw quiet. Acceptance requires at least five peak observations,
  at least 500 ms between first/last peak, duration no longer than 30 seconds,
  and an actual MVS-3 confirmation during the episode. Held ON alone is not a
  confirmation. Each accepted episode contributes one mean-energy vote.
* **Sensitivity/recurrence:** each bin's movement-to-quiet ratio is capped at
  20. The recurrence measure is the proportion of accepted episodes in which
  that bin exceeds four times its quiet energy (with an absolute noise floor).
  Movement and recurrence averages have a capped 16-episode time scale.

Candidate weight = `0.75 + 0.15 * silence + 0.30 * response + 0.30 * recurrence`,
clamped to `[0.75, 1.50]`. Silence is relative cleanliness, response is the
episode ratio divided by 10 (capped at 1), recurrence lies in `[0,1]`.
Initial weights are all 1. Each committed change is limited to 0.05 per bin.

Every revision requires **three new accepted episodes and six new quiet blocks**
since the preceding revision, at least five minutes since that revision (or
boot), and completion of a quiet block outside an episode and outside the
MVS-3 hold. Repeated quiet or time alone cannot repeatedly apply old evidence.
This is intentionally conservative: a busy/noisy room may never earn a revision.
Boot should include a quiet period. There is no externally verified movement
ground truth; CSI corroboration and bounded updates reduce, but do not prove
absence of, environmental classification errors. Nothing is persisted on reboot.

## Jitter and recalibration

Experimental jitter is `sqrt(sum(weight[i] * raw_energy[i]) / sum(weights) + 1e-4)`.
After a revision, quiet jitter is projected from the learner's **unweighted**
per-bin quiet energies using the same formula. The confirmation filter and
previous profile are cleared. The next observation seeds the profile without
making a decision, so the change cannot manufacture a temporal jump. Accumulation
has already completed at that point. A CSI gap or parsed-layout change discards
partial observations/episodes; it does not count as movement evidence.

## Minute log

One INFO line every 60 seconds in the main loop, even if CSI has stopped:

`MVS3 learn q=... ep=... reject=... rev=... hi=bin:candidate/weight lo=bin:candidate/weight`

`q` counts complete quiet blocks; `ep` counts accepted episodes; `reject` counts
rejected/aborted episodes; `rev` counts committed revisions. Bin indices are
zero-based. `hi`/`lo` show extrema of the candidates; all 48 weights remain active.
The loop reads atomic diagnostic scalars, never the callback-owned learner.

## Compile and run

```sh
git clone --branch feature/mvs3-raw-episode-learning https://github.com/lbarreiro/esphome-esp-wifi-sensing.git
cd esphome-esp-wifi-sensing
python -m pip install esphome pytest
```

Create `examples/secrets.yaml` locally (it is ignored by Git):

```yaml
wifi_ssid: "your-network"
wifi_password: "your-password"
```

Then:

```sh
esphome config examples/mvs_three_branches.yaml
esphome compile examples/mvs_three_branches.yaml
esphome run examples/mvs_three_branches.yaml
```

The example uses `esp32dev` with ESP-IDF. Adapt board/network settings to your
hardware. In an existing YAML, use the same component from this work branch and
add `motion_3` alongside `motion` and `motion_128`; keep your existing MVS settings.
For a remote external component, select this branch explicitly:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/lbarreiro/esphome-esp-wifi-sensing.git
      ref: feature/mvs3-raw-episode-learning
    components: [esp_wifi_sensing]
    refresh: 0s
```

## Tests

Existing regression model: `python -m pytest tests/test_mvs_algorithm.py`.
The new test executes the real learner and MVS implementations with a host C++
compiler, rather than a Python copy of their logic:

```sh
c++ -std=c++17 -Wall -Wextra -Werror -Itests/stubs -Icomponents/esp_wifi_sensing tests/test_learning.cpp components/esp_wifi_sensing/mvs_algorithm.cpp components/esp_wifi_sensing/experimental_mvs_algorithm.cpp -o test_learning
./test_learning
```

The test Wi-Fi header is a host-only type stub and is not part of the firmware.
Hardware validation should compare the three sensors in a quiet room and over
repeated separated movements; observe evidence counts before expecting weights
to change. Build success alone does not establish sensing accuracy.

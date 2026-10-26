# MVS-256 reference and MVS-3 learned detector

Branch: `feature/mvs3-raw-episode-learning`.

## Two independent detection paths

Both detectors use the existing Wi-Fi CSI radio and the same ping session.
No additional ping traffic is created.

1. **MVS1 / MVS-256 reference (`motion`)**: the existing gain-compensation
   pipeline and existing parser feed a private `MvsAlgorithm` instance with
   its own temporal profile, quiet jitter, confirmation filter, and ON hold.
   The reference algorithm implementation (`mvs_algorithm.h/.cpp`) is unchanged.
2. **MVS-3 / MVS-256 learned (`motion_3`)**: a separate `CsiParser` parses
   the original 256-byte CSI packet before optional gain compensation.
   A separate `ExperimentalMvsAlgorithm` instance owns its own observation
   accumulator, temporal profile, filter, quiet calibration, ON hold, and
   `CsiLearning` state. Its weights never flow into MVS1.

The shared configuration setters supply the *same configured parameters* to
both independent detectors. The radio, packet source and callback scheduling are
necessarily shared; algorithm state and decisions are **not** shared.
The C++ regression test processes identical packets through MVS1 both alone
and alongside an adapting MVS-3, comparing every returned score and state.
This is a software isolation check, not a guarantee of zero shared CPU load.

The unused **MVS2 / CSI-128 comparison detector has been removed**: no parser
instance, algorithm instance, state, binary sensor, or 128-byte comparison
processing remains. A passive packet-length counter for 128-byte CSI may still
appear in INFO logs; it is a diagnostic counter, not a detector.

## MVS-3 learning

* 48 bins, all retained in the weighted jitter calculation; allowed weights
  remain in `[0.70, 2.00]` and begin at `1.00` on boot.
* Long-term quiet baselines are built from completed five-second blocks with
  enough valid raw-quiet observations. Quiet updates alone do not revise
  candidate weights.
* A raw movement episode must be confirmed by MVS-3, meet minimum duration
  and count conditions, and finish with at least two seconds of raw quiet.
* A learning-eligible episode also needs a fresh eight-observation pre-motion
  baseline and separation from previous eligible episodes. Each subcarrier
  must be quiet before that episode, exceed its individualized threshold in
  at least three observations and for at least 25% of its peak observations,
  and reach a mean movement-to-quiet ratio of at least 4x.
* Only **5 or more valid transitions in the last 8 eligible episodes** earn a
  positive candidate. Relative movement strength is weighted heavily; its
  logarithmic contrast term preserves differences from 4x to 256x.
* Candidate revision is allowed only at a completed quiet block, with at
  least three newly accepted episodes, at least six newly accepted quiet
  blocks, and at least five minutes since the prior revision. A new revision
  applies **90% of the gap** to the candidate, with absolute clamping:

  `weight = clamp(weight + 0.90 * (candidate - weight), 0.70, 2.00)`

The initial 8 eligible episodes and the 5/8 rule are unchanged by the 90%
convergence choice. No learning state is persisted across firmware restarts.

After a committed weight update MVS-3 independently reprojects its quiet
reference and clears its filter/previous profile to prevent a synthetic
temporal jump. This does not recalibrate or reset MVS1.

## Four-bin priority (70/30)

The priority decision is **inactive** until at least four bins qualify under
the existing rolling `5/8` verified quiet-to-motion recurrence rule. At a
guarded weight revision, MVS-3 deterministically chooses the four highest
**qualified** candidates; a tie favors the lower bin index. The selection is
held constant until a later guarded revision (no changes from isolated new
episodes). If fewer than four qualify on a subsequent revision, the priority
path is disabled.

Once enabled, global MVS-3 decision energy is
`0.70 * weighted_mean(energy[top4]) +
0.30 * weighted_mean(energy[other44])`. Each group is separately normalized
by its total learned weight. **Before qualification**, the global score is the
original 48-bin weighted mean.

In addition, the four selected bins have their own score, independently
calibrated using their own learned quiet energies, and an independent rolling
motion confirmation filter. Like the global filter, this requires three hits
in eight observations for OFF-to-ON and two in eight to renew ON. The final
MVS-3 decision is `global_confirmed OR top4_confirmed`; therefore the four
can trigger ON even when the other 44 are unresponsive. Both paths share the
same 120-second output hold, but no filter state. Changes of priority
selection, a CSI gap or a weight revision reset their confirmation history
rather than causing a synthetic ON. This experiment can increase sensitivity
and also false positives, so evaluate against known empty-room periods.

The global MVS3 score (`diag score`) and four-bin score (`priority score`)
have their own calibrated quiet references and use the configured threshold
without comparing raw energy of 4 bins directly to energy of 48 bins.

## Logs

* `MVS3 learn q=... ep=... reject=... rev=... hi=bin:candidate/weight lo=...`
* `MVS3 diag obs=... held=... score=... energy=... limit=... rawquiet=...`
* `MVS3 bins win=8/8 hit5=... up=... strong=... weak=... mean=...`
* `MVS3 gate eligible=... skipped=... prequiet=... transitions=...`
* `MVS3 priority active=... split=70/30 bins=a,b,c,d score=... confirms=...`

`active=0` means the 48-bin metric is still used and no priority confirmation
is attempted. `active=1` means four qualified bins were selected at a
revision; `confirms` counts confirmations from that path, including renewals.
A low global score does **not** preclude motion when the priority score
and its own filter confirm ON.

The main ESPHome loop reads atomic snapshots of MVS-3 diagnostics; it never
directly reads or mutates callback-owned learning arrays.

## Update YAML

In your own ESPHome YAML, keep the independent `motion` and `motion_3`
binary sensors, but **delete the entire `motion_128:` block**, otherwise
ESPHome configuration validation will reject the obsolete key:

```yaml
binary_sensor:
  - platform: esp_wifi_sensing
    esp_wifi_sensing_id: wifi_sensing
    motion:
      name: CSI Motion 256 Reference
    motion_3:
      name: CSI Motion 256 Learned
```

The existing example filename `examples/mvs_three_branches.yaml` is kept
unchanged so automated build paths continue working; its contents now describe
only the two supported detectors.

The external component branch remains:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/lbarreiro/esphome-esp-wifi-sensing.git
      ref: feature/mvs3-raw-episode-learning
    components: [esp_wifi_sensing]
    refresh: 0s
```

## Validation

GitHub Actions runs the Python reference regression suite, host C++ tests,
ESPHome YAML validation, and a full ESP-IDF firmware build.
The C++ tests verify 90% convergence, bounded weights, quiet-only stability,
recurrent transitions, and the independence of MVS1 from MVS-3 learning.

Build success is not proof of higher real-world detection accuracy. Compare
the ON/OFF traces of the two sensors against known movement/empty-room periods.

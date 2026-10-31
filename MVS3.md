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

## Two-bin priority (70/30)

The priority decision is **inactive** until at least two bins qualify under
the existing rolling `5/8` verified quiet-to-motion recurrence rule. At the
end of every **new eligible movement episode**, MVS-3 chooses the two highest
**qualified** candidates; a tie favors the lower bin index. Selection no
longer waits for a guarded weight revision: `rev` only updates the learned
weights. When either selected bin falls below 5/8, MVS-3 immediately replaces
the pair if two qualified alternatives exist; otherwise the 70/30 priority is
disabled and the ordinary weighted 48-bin detector remains active.
A previously disabled priority automatically reactivates as soon as two
qualify, without waiting for a revision. A single qualified bin is not enough.
When both currently selected bins **remain qualified**, a replacement pair
must beat the current pair's weakest candidate by at least 0.05 and stay
best-ranked for **two consecutive eligible episodes**, avoiding frequent
switches from small ranking fluctuations. No changes occur during silence
alone. A selection change clears MVS-3's temporal profile and confirmation
filters and recalibrates its own quiet references, without resetting the
existing 120-second ON hold or any MVS1 state.

Once enabled, global MVS-3 decision energy is
`0.70 * weighted_mean(energy[top2]) +
0.30 * weighted_mean(energy[other46])`. Each group is separately normalized
by its total learned weight. **The global 70/30 metric is applied to every
scored observation whenever a qualified Top 2 exists**. It no longer requires
both selected bins to respond at the same instant: even if one bin is quiet,
the global detector still uses the 70/30 formula. **Before qualification**
or whenever the Top 2 becomes inactive, the global score uses the ordinary
weighted 48-bin mean. The global 70/30 and 48-bin metrics retain separate
quiet references to avoid artificial jumps when the selected pair changes.

In addition, the two selected bins have their own score, independently
calibrated using their own learned quiet energies, and an independent rolling
motion confirmation filter. **This independent confirmation path is unchanged**:
an observation counts as a priority hit only if BOTH bins independently
respond at that moment AND the joint score meets the configured threshold.
Like the global filter, this requires three
hits in eight observations for OFF-to-ON and two in eight to renew ON. The final
MVS-3 decision is `global_confirmed OR top2_confirmed`; therefore the two
can trigger ON even when the other 46 are unresponsive, but neither can
trigger the priority path alone. Both paths share the same 120-second
output hold, but no filter state. Selection changes, priority suspension, CSI
gaps or weight revisions reset their confirmation history and seed the
temporal profile to avoid synthetic ON. An existing ON hold is allowed to
expire normally after suspension. Evaluate real empty-room false positives
against the unchanged MVS1 reference.

The global MVS3 score (`diag score`) and two-bin score (`priority score`)
have their own calibrated quiet references and use the configured threshold
without comparing raw energy of 2 bins directly to energy of 48 bins.

The 70/30 percentages apply to the **two groups' decision energy**, not
to the individual learning coefficients. There are still 48 learned weights
and all 48 bins contribute to the global decision.

## Large topic headers, every 60 seconds

The existing **60-second interval** is preserved. Each line starts directly
with a clear uppercase topic (`ESTADO:`, `TOP 2:`, `MIN/MAX:`,
`APRENDIZAGEM:` or `DIAGNOSTICO:`), without repeated `MVS3` prefixes or
`===` decorations. ESPHome provides the source tag separately; the firmware
does **not** control the font size in the ESPHome log viewer.

1. `ESTADO:`: currently published movement ON/OFF; whether a
   **qualified Top 2 is available** (not synonymous with applying 70%);
   how many seconds since the last actual confirmation; that confirmation's
   route (`GLOBAL48`, `GLOBAL70`, `TOP2`, `AMBOS`, or `SEM`);
   and `70_USADO=A/B (P%)`, the **A observations in the last minute**
   that actually applied 70/30 out of B scored observations. When no
   movement has ever been confirmed, the last confirmation says `SEM`.
2. `TOP 2:`: selected bin numbers, each bin's hits out of 8,
   candidate (`cand`) and committed learned weight (`peso`).
   Also the number of 5/8-qualified bins and eligible episode window.
   When no pair is qualified it says `INATIVO (48 PORTADORAS)`.
3. `MIN/MAX:`: best and worst current candidate bins,
   recurrence, candidate, weight, and promotion/quality counts.
   Because hysteresis stabilizes the selected pair, the best candidate
   can differ from a selected Top 2 bin.
4. `APRENDIZAGEM:`: committed weight revision `rev`,
   accepted/eligible/skipped/rejected episodes, accepted quiet blocks,
   and latest prequiet/transition/10x/256x counts.
5. `DIAGNOSTICO:`: global and independent Top 2 scores,
   cumulative priority confirmations, retained motion state, raw quiet
   classification, observed energy and limit, sample counts, dropped
   blocks, hold resets and CSI discontinuities.

The event log for a transition to ON or OFF is also labeled `ESTADO` and
includes the last detection route and the existing 120-second hold. A global
confirmation is labeled `GLOBAL70` if 70/30 was applied to the confirming
observation, or `GLOBAL48` when the 48-bin metric was used.
`AMBOS` means the global and independent priority paths confirmed
simultaneously. This is the **last** confirmed route, not a guarantee that
the current observation is still above the movement threshold.

Individual Top 2 hits are from the last eight eligible episodes;
`cand` can differ from `peso` because the latter changes only at `rev`.
The number of qualified bins counts bins meeting the 5/8 rule, not
eligible episodes. The cumulative `confirms` count includes hold renewals,
not just distinct human movements. The independent CSI packet-length
diagnostic is unchanged.

With the unrestricted global 70/30 weighting, `70_USADO` now reports
how many scored observations happened while a qualified Top 2 existed;
it no longer measures simultaneous reactions of the two individual bins.
It will typically approach 100% during uninterrupted Top 2 qualification,
**not** indicate motion accuracy or simultaneous response. Independent Top 2
confirmations still require both bins to react in the same observation.

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
recurrent transitions, top-2 qualification, single-bin noise rejection,
same-observation joint confirmation, fallback to the calibrated 48-bin
metric, automatic replacement/reactivation after eligible episodes without
a weight revision, and the independence of MVS1 from MVS-3 learning.

Build success is not proof of higher real-world detection accuracy. Compare
the ON/OFF traces of the two sensors against known movement/empty-room periods.

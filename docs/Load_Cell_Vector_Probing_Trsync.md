# Load-cell vector probing and trsync

This note captures the remaining design issue around using the load-cell probe
for X/Y or arbitrary vector probing.

## Problem

`trsync` stops the steppers that are registered with the probing endstop when a
probe trigger occurs. Traditional probes are Z-only, so their wrappers register
only Z steppers. A load-cell probe used for tool calibration is different: the
same force trigger may need to stop X, Y, Z, CoreXY, or other kinematic steppers
depending on the current probe vector.

For example:

- `PROBE` with no explicit target should behave like a normal Z probe.
- `PROBE X=50` should stop the steppers involved in X motion.
- `PROBE X=50 Y=50` should stop the steppers involved in that diagonal move.
- Future pin probing may use arbitrary approach vectors, including non-axis
  aligned vectors.

The key requirement is that both the MCU trigger and host-side trigger-position
reconstruction agree on the same active steppers for the current probe move.

## Why the existing code is inadequate

The current probe/endstop model assumes the set of probe steppers is static.
That works for ordinary Z probes, but not for a vector-capable load-cell probe.

There are two simple static choices, and both are wrong for some vector probes:

1. **Register only Z steppers.**
   - Existing Z probing works.
   - X/Y vector probes are not properly stopped or reconstructed because the
     moving X/Y steppers are not part of the probe endstop.

2. **Register every kinematic stepper.**
   - The probe trigger can stop X/Y/Z motion.
   - However, `HomingMove` also uses `get_steppers()` to build the stepper list
     used for trigger-position reconstruction and `check_no_movement()`.
   - Stationary steppers in that list can appear to have no movement at trigger
     time, producing false "Probe triggered prior to movement" failures.

In other words, the existing static interface conflates two related concepts:

- which steppers this probe may ever need to stop; and
- which steppers are active for this specific probe vector.

Vector probing needs the second concept to be scoped per move.

## Approach 1: persistent trsync signals with an active mask

The most complete solution is to split trsync signal registration from per-move
activation.

Today, `trsync_signal` entries are effectively one-shot. The host sends
`stepper_stop_on_trigger` for each stepper before each homing/probing move, and
`trsync_do_trigger()` consumes the signal list when the trigger fires. A
vector-capable probe would be cleaner if the MCU could instead keep a persistent
list of possible signals, with a per-move active set.

Conceptually:

```c
struct trsync_signal {
    struct trsync_signal *next;
    trsync_callback_t func;
    uint8_t index;
};

struct trsync {
    struct trsync_signal *signals;
    uint32_t active_mask;
};
```

Each stepper that may ever need to stop for this `trsync` would be registered
once and assigned a signal index. Before each probe move, the host would send a
single active mask describing which registered signals are active for that move.
When the probe triggers, the MCU would call only signal callbacks whose bit is
set in the active mask.

This is the most robust long-term model because it makes the two separate states
explicit:

- persistent configured membership: every stepper this probe may ever need to
  stop;
- per-move active membership: the steppers involved in this exact probe vector.

The main benefit is that the per-move cost can be one small command instead of
one command per active stepper. That matters for repeated commands such as
`PROBE_ACCURACY SAMPLES=10`.

The tradeoff is that this is a firmware protocol change. `trsync_do_trigger()`
and `trsync_clear()` would need to stop consuming/removing persistent signals,
and the host would need a compatible way to register signals and set the active
mask. The design also needs a fallback for machines with more steppers than fit
in one mask.

## Approach 2: host-side active stepper subset with current trsync

The smaller implementation is to keep the current one-shot `trsync_signal` model
and change only the host-side arming step.

Before each probe move, the host would calculate the active stepper subset from
the current vector and pass that subset into `TriggerDispatch.start()`. The
existing `stepper_stop_on_trigger` command would then be sent only for active
steppers.

Conceptually:

```python
active_steppers = find_steppers_that_move(start_pos, target_pos)
trigger_dispatch.start(print_time, active_steppers)
```

This preserves the existing MCU firmware behavior:

- `trsync_signal` remains one-shot;
- `trsync_do_trigger()` still consumes the signal list;
- no MCU command protocol changes are required.

The main benefit is low implementation risk. It scopes the MCU-side stop list to
the current vector and lets `HomingMove` use the same active stepper list for
trigger-position reconstruction and `check_no_movement()`.

The tradeoff is that it still sends one `stepper_stop_on_trigger` command per
active stepper per probe move. That is probably acceptable for typical
Cartesian/CoreXY machines, but it is less efficient than the persistent-mask
approach and still requires a generic host-side API change in `TriggerDispatch`
or nearby code.

## Approach 3: reusable active-axis / active-stepper move context

A third option is to package the per-move vector interpretation into a small
host-side object that can be shared by load-cell probing and future
vector-capable probes.

The core observation is that the trsync problem is downstream of one reusable
question: given a probe move from `start_pos` to `target_pos`, which toolhead
axes are active, and which kinematic steppers does that imply? If that decision
lives in a dedicated object, both `HomingMove` and the probe/trsync arming code
can consume the same answer instead of each probe wrapper reimplementing its own
filtering.

Conceptually:

```python
move_context = ProbeMoveContext(toolhead, start_pos, target_pos)

active_axes = move_context.get_active_axes()
active_steppers = move_context.get_active_steppers(candidate_steppers)

homing_move = HomingMove(printer, endstops, toolhead, move_context)
trigger_dispatch.start(print_time, active_steppers)
```

The object would be responsible for the reusable move classification:

- compare the start and target toolhead positions using the same tolerances as
  the rest of the homing/probing path;
- expose the active toolhead axes for the move, for example `Z` for ordinary
  probing, `X` for side probing, and `X/Y` for a diagonal move;
- map those active axes through the current kinematics to determine which
  steppers should be considered active for trigger-position reconstruction;
- filter any candidate stepper list supplied by a probe endstop wrapper;
- optionally expose the same active stepper subset to `TriggerDispatch` so MCU
  stop-on-trigger arming and host-side reconstruction stay in lock-step.

That keeps the policy separate from the probe hardware. A load-cell probe can
register all kinematic steppers as possible trigger targets, while the move
context decides which ones are active for the current move. A future probe that
supports arbitrary approach vectors could reuse the same object and only supply
its own candidate steppers or vector constraints.

The main benefit is that it makes the missing concept explicit without requiring
an immediate MCU protocol change. It is more reusable than putting transient
state directly in the load-cell wrapper, and it creates a natural place for
future vector-probing rules such as minimum movement thresholds, non-axis-aligned
approaches, or kinematics-specific stepper selection.

The tradeoff is that this still needs careful integration with the existing
homing API. `HomingMove` is constructed around endstops whose `get_steppers()`
method is currently static, so the move context either needs to be passed into
`HomingMove`, passed into the endstop wrapper before construction, or made
available through a small host-side API that `TriggerDispatch` can also use. It
also does not by itself make trsync more efficient; unless combined with the
persistent-mask design, the current MCU still receives one
`stepper_stop_on_trigger` command per active stepper per move.

## Proposed solution

Add a small dynamic active-stepper path for probe moves:

1. Before starting a probe move, calculate the current probe vector from the
   toolhead start position to the requested target position.
2. Store that vector decision in a reusable probe move context that exposes the
   active toolhead axes and derived active stepper subset.
3. Use kinematic stepper coordinate transforms to determine which registered
   steppers will actually move for that vector.
4. Expose that active-stepper subset to `HomingMove.get_steppers()` so trigger
   position reconstruction and `check_no_movement()` only consider steppers that
   should move.
5. Ideally, also pass the same active-stepper subset into `TriggerDispatch` /
   `MCU_trsync` so `stepper_stop_on_trigger` is only armed for steppers involved
   in this probe move.

The load-cell probe can still register all kinematic steppers as potential
probe steppers during MCU identify. The active subset should be selected per
probe move, not permanently. Keeping that selection in a move-context object
would make the same logic available to future vector-capable probes instead of
binding it to the load-cell implementation.

This keeps normal Z probing compatible while allowing X/Y/diagonal probing to
use the same force trigger and the same homing/trsync machinery.

## Next best alternative

If we want to avoid changing the generic `TriggerDispatch` / `MCU_trsync` API,
the next best alternative is to keep the dynamic filtering entirely in the
load-cell wrapper:

- register all kinematic steppers as potential load-cell probe steppers;
- have the wrapper return only the current vector's active steppers from
  `get_steppers()` while `HomingMove` is being constructed;
- reset that active-stepper state when the probe move ends or errors.

This avoids broad MCU-layer changes and fixes host-side reconstruction and false
no-movement checks. The tradeoff is that it does not provide a generic trsync
subset API, so it is less reusable for future vector-capable probe types and may
still need revisiting if we find that MCU-side stop arming must also be scoped
more tightly per move.

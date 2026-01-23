# Toyota Low‑Speed Cruise Override (FrogPilot/Openpilot) — Agent Guide

## Non‑negotiables (keep this file and `agents.md` in **parity**)
- **Parity requirement:** `Claude.md` and `agents.md` must always be **identical**. Update both immediately, every time.
- **Contradictions:** If anything is found that contradicts these instructions, **rectify and update immediately** so the guidance is correct and unambiguous.
- **Missing info:** If relevant information is discovered that is not in **either** document, **add it immediately** to both.
- **Scope:** Focus on Toyota (especially 2017 Corolla and Toyota platforms needing gas‑interceptor/comma pedal for stop‑and‑go). Avoid exploring `selfdrive/car/**` unless absolutely unavoidable.
- **Ignore prior override flag:** Treat any existing `toyota_low_speed_override` flag as **non‑existent** for this guidance.
- **Units:** Use **imperial (mph)** for user‑facing descriptions; convert internally as needed (kph in code).
- **Safety model:** Adhere to openpilot safety expectations and minimize behavioral risk.
- **Smallest footprint:** Prefer minimal, isolated changes; avoid broad refactors.

## Objective (summary)
Implement a **Toyota low‑speed cruise override** that allows **set speeds below the PCM/DSU floor (~28 mph)** using FrogPilot's existing speed‑limit tooling and cruise logic. Preserve all stock/FrogPilot behaviors (speed limit controller, rounding, hold‑press cadence) while seamlessly handing off between **PCM‑controlled cruise** and **OP‑owned** low‑speed setpoints with no flicker or latency.

## Canonical driver‑input semantics (clarified)
- **Cruise stalk orientation:** On Toyota, the stalk is right of the wheel.
  - **Up / + (increase):** press toward the sky.
  - **Down / – (decrease):** press toward the floor.
  - **Toward driver:** pull toward the driver.
  - **Away from driver:** push toward the engine.
- **Main‑on, not engaged:**
  - **RES/+** should resume previous set speed (even if below 28 mph).
  - **SET/–** should set to current speed (even if below 28 mph).
- **Tap vs hold:**
  - Tap makes a single increment; hold repeats at the stock/FrogPilot cadence.
  - FrogPilot toggle can switch increments from ±1 to ±5; rounding behavior must be preserved.

## Existing key behaviors & constraints to preserve
- **Toyota PCM floor:** Toyota PCM/DSU won't report/set below ~28 mph; OP must own the set speed below that floor. (`V_CRUISE_PCM_FLOOR` in `drive_helpers.py`).
- **Button semantics:**
  - **Tap** vs **hold** behavior must be consistent with FrogPilot (including the custom ±5 toggle and rounding).
  - **Resume vs set from main‑on:** When main cruise is on but not engaged, **RES/+** should resume prior set speed. **SET/–** should set to current speed. Below floor, the override must take control immediately and update the displayed set speed accordingly.
- **SLC interaction:** If SLC provides a new limit (e.g., 10 mph) while override is active, behave like stock FrogPilot: SLC should clamp/limit the effective target speed.
- **Display stability:** Avoid flicker between PCM and override speeds; when override is active the displayed set speed should be stable and match the OP‑owned value.
- **No delays:** Transitions between PCM and override must be seamless (no extra latency vs stock).
- **Toyota family constraints:** Respect any Toyota variants that already allow below‑floor set speeds; do not override lower OEM limits if they are already lower than the Corolla's floor.

## Relevant code locations (Toyota‑focused & generic only)
> **Do not inspect `selfdrive/car/**` unless absolutely required.**

### 1) Cruise set speed handling + button timing
- **`selfdrive/controls/lib/drive_helpers.py`**
  - `V_CRUISE_PCM_FLOOR`: Toyota PCM floor in kph (~28 mph).
  - `VCruiseHelper.update_v_cruise(...)`: entry point for cruise set‑speed updates (PCM and non‑PCM).
  - `CRUISE_LONG_PRESS`, `CRUISE_NEAREST_FUNC`, `CRUISE_INTERVAL_SIGN`: core button handling/rounding logic.
  - `update_button_timers(...)`: tracks tap vs hold transitions and stores standstill/enabled state for button edges.
  - **Notes:** Implement the new low‑speed override as a **new helper** under `VCruiseHelper`, mirroring the existing rounding and long‑press cadence used for cruise set speed changes.

### 2) Cruise update flow in controlsd
- **`selfdrive/controls/controlsd.py`**
  - `v_cruise_helper.update_v_cruise(...)` is called during state transitions using `frogpilotPlan.speedLimitChanged` to suppress accidental speed changes when SLC prompts.
  - `controlsState.vCruise` & `controlsState.vCruiseCluster` are published from `VCruiseHelper` and consumed by UI and FrogPilot planners.

### 3) FrogPilot vCruise aggregation (SLC + curve speed)
- **`frogpilot/controls/lib/frogpilot_vcruise.py`**
  - Combines **Curve Speed Controller** and **Speed Limit Controller** outputs with cruise set speed.
  - Uses `vCruiseCluster`/`vCruise` deltas to compute overrides and blend speeds.
  - **Important:** SLC target and overrides ultimately clamp the `v_cruise` target used downstream.
- **`frogpilot/controls/lib/curve_speed_controller.py`**
  - Computes curve‑based target speeds that can further limit `v_cruise`; ensure override speeds still respect curve limiting.

### 4) Speed Limit Controller (SLC)
- **`frogpilot/controls/lib/speed_limit_controller.py`**
  - `handle_limit_change(...)`: confirmation logic that uses accel/decel press signals.
  - `update_limits(...)`: priority source logic (Dashboard/Map/Navigation/Mapbox) and fallback modes.
  - `update_override(...)`: gas‑pedal vs set‑speed override behavior.
  - **Important:** SLC can override to a lower target—this must still affect the low‑speed override.

### 5) FrogPilot toggle definitions + params
- **`frogpilot/common/frogpilot_variables.py`**
  - `cruise_increase`, `cruise_increase_long`, `reverse_cruise_increase`, `set_speed_offset`: cruise adjustment behavior.
  - `speed_limit_controller` and SLC configuration toggles.
- **`common/params.cc` + `frogpilot/common/frogpilot_variables.py` (param list + defaults)**
  - SLC params: `SLCFallback`, `SLCConfirmation*`, `SLCLookahead*`, `SLCOverride`, `SLCPriority*`.
  - Cruise interval params: `CustomCruise`, `CustomCruiseLong`, `SetSpeedOffset`, `ReverseCruise`.

### 6) FrogPilot planner + longitudinal planner
- **`frogpilot/controls/frogpilot_planner.py`**
  - Produces `frogpilotPlan.vCruise` via `FrogPilotVCruise.update(...)` and publishes `speedLimitChanged` flag.
  - Exposes SLC fields such as `slcSpeedLimit`, `slcSpeedLimitOffset`, and `slcOverriddenSpeed` for UI/logic consumption.
- **`selfdrive/controls/lib/longitudinal_planner.py`**
  - Consumes `frogpilotPlan.vCruise` as the effective cruise target for longitudinal planning.

### 7) UI set‑speed display
- **`selfdrive/ui/qt/onroad/annotated_camera.cc`**
  - Draws the **MAX set speed** box; uses the set speed from UI state derived from `controlsState`.
  - Handles older routes where `vCruiseCluster` may not be set.
  - **Critical for flicker:** Keep `controlsState.vCruise` and `controlsState.vCruiseCluster` aligned to the OP‑owned value while override is active.

### 8) FrogPilot tracking + stats (contextual)
- **`frogpilot/system/frogpilot_tracking.py`**
  - Uses `controlsState.vCruiseCluster` to track cruise speed usage; avoid introducing inconsistent values that could skew stats.

### 9) Controls state + events plumbing (generic but relevant)
- **`selfdrive/controls/lib/events.py`**
  - Event handling and gating that can affect enable/disable and user‑visible alerts.
- **`selfdrive/controls/lib/longcontrol.py`**
  - Longitudinal control state machine; relevant for understanding when speed targets are respected or ignored.

### 10) Core utilities & data definitions used by cruise logic
- **`common/conversions.py`**
  - Unit conversions (mph↔kph, kph↔m/s). Ensures imperial‑facing behavior is converted correctly for internal logic.
- **`common/realtime.py`**
  - Timing constants such as `DT_CTRL` and `DT_MDL` that define button cadence and control loop timing.
- **`common/numpy_fast.py`**
  - `clip`/`interp` utilities used for bounded speed changes and rounding behavior in cruise logic.
- **`cereal/car.capnp`**
  - Defines `CarState.ButtonEvent` and button types used for cruise stalk interpretation.

## Verifiable repo facts to add to the plan (keep updated)
- **Tap/hold cadence:** `CRUISE_LONG_PRESS = 50` in `drive_helpers.py` controls when a press becomes a hold‑repeat event.
- **Cruise floors/limits:** `V_CRUISE_MIN`, `V_CRUISE_MAX`, and `V_CRUISE_UNSET` are defined in `drive_helpers.py` and bound set‑speed logic.
- **Imperial increment:** `IMPERIAL_INCREMENT` in `drive_helpers.py` is derived from mph→kph conversion and used for step sizing.
- **Speed‑limit confirm/deny guard:** `drive_helpers.py` skips speed adjustments when `frogpilotPlan.speedLimitChanged` is active, so new logic must honor this guard.
- **Hold detection cadence:** `update_button_timers(...)` increments per control tick and drives long‑press repeat scheduling (no separate debounce helper).
- **SLC prompt suppression:** `frogpilotPlan.speedLimitChanged` is used in `controlsd` to avoid adjusting speed when SLC confirmation is pending.
- **UI fallback:** `annotated_camera.cc` explicitly handles older routes where `vCruiseCluster` is unset; ensure override logic does not regress this.
- **Custom cruise increments:** `frogpilot_variables.py` gates `cruise_increase` / `cruise_increase_long` behind QoL longitudinal and non‑PCM cruise, so below‑floor logic must respect those gating rules.
- **Reverse cruise behavior (Toyota PCM):** `frogpilot_variables.py` gates `reverse_cruise_increase` behind Toyota + PCM cruise and the `ReverseCruise` param.
- **SLC override mode gating:** `frogpilot_variables.py` maps `SLCOverride` to `speed_limit_controller_override_manual` / `speed_limit_controller_override_set_speed` booleans.
- **SLC confirm/deny inputs:** `speed_limit_controller.py` accepts via `accelPressed` + `longActive` or `SpeedLimitAccepted` param and denies via `decelPressed` or a 30‑second timeout.
- **v_cruise_cluster clamp:** `frogpilot_vcruise.py` computes `v_cruise_cluster = max(controlsState.vCruiseCluster, v_cruise)` before calculating SLC override deltas.
- **SLC activation gating:** `frogpilot_variables.py` sets `speed_limit_controller` only when `openpilot_longitudinal` is true and `SpeedLimitController` is enabled (per tuning level).
- **UI set‑speed selection:** `annotated_camera.cc` uses `vCruiseCluster` when non‑zero, otherwise falls back to `vCruise` for the MAX speed display.
- **SLC override trigger:** `speed_limit_controller.py` enables override when `overridden_speed` exceeds `target + offset` or when `gasPressed` drives `v_ego` above `target + offset`, and only while `controlsState.enabled`.
- **SLC override source handling:** `speed_limit_controller.py` sets `source = "None"` while override is active and resets `overridden_speed` to `0` when override is inactive.
- **v_cruise target selection:** `frogpilot_vcruise.py` builds a `targets` list (curve target, set speed, and SLC target/override minus `v_ego_diff`) and picks the minimum target that is at least `CRUISING_SPEED`; otherwise it falls back to current `v_cruise`.
- **v_ego_cluster clamp:** `frogpilot_vcruise.py` computes `v_ego_cluster = max(vEgoCluster, v_ego)` and uses `v_ego_diff` when applying SLC override deltas.
- **SLC manual override clipping:** `speed_limit_controller.py` clips `overridden_speed` between `target + offset` and `v_cruise + v_cruise_diff` when manual override is active.
- **Set‑speed‑limit on engage:** `drive_helpers.py` uses `frogpilot_toggles.set_speed_limit` to initialize `v_cruise` to the current speed limit when engaging.
- **SetSpeedLimit gating:** `frogpilot_variables.py` sets `set_speed_limit` only when `speed_limit_controller` is enabled and the `SetSpeedLimit` param is on (per tuning level).
- **SLC fallback modes:** `frogpilot_variables.py` maps `SLCFallback` into `slc_fallback_experimental_mode`, `slc_fallback_previous_speed_limit`, and `slc_fallback_set_speed` booleans.
- **SLC mapbox requests:** `speed_limit_controller.py` initializes a monthly `max_requests` budget based on `FREE_MAPBOX_REQUESTS` minus daily allowance and increments `MapBoxRequests` on each request.
- **SLC fields in FrogPilotPlan:** `cereal/custom.capnp` defines SLC fields like `slcSpeedLimit`, `slcSpeedLimitOffset`, `slcOverriddenSpeed`, and `slcSpeedLimitSource` used for UI/logic.
- **Speed limit changed flag:** `cereal/custom.capnp` includes `speedLimitChanged`, which is set in `frogpilot_planner.py` when SLC state changes.
- **SLC mapbox failure behavior:** `speed_limit_controller.py` resets `mapbox_limit` to `0` and sets `segment_distance = v_ego` on request failures (or 1000 when host is unreachable), throttling follow‑up requests.
- **Map speed lookahead gating:** `frogpilot_variables.py` sets `map_speed_lookahead_higher/lower` only when `speed_limit_controller` is enabled and the tuning level allows it.
- **SLC priority resolution:** `speed_limit_controller.py` selects speed limits by highest/lowest or priority list order, and falls back to Mapbox only when no source provides a limit (or when target is zero).
- **SLC fallback to previous/set speed:** When no limit is available, `speed_limit_controller.py` can fall back to the previous limit (`slc_fallback_previous_speed_limit`) or to current set speed (`slc_fallback_set_speed`) if enabled and controls are active.
- **Cruise rounding behavior:** `drive_helpers.py` uses `CRUISE_NEAREST_FUNC` (`ceil` for accel, `floor` for decel) when rounding to ±5 intervals so tap rounding matches FrogPilot behavior.
- **Cruise step direction:** `drive_helpers.py` maps accel to `+1` and decel to `-1` via `CRUISE_INTERVAL_SIGN`, which is applied to both tap and hold increments.
- **Set‑speed offset on long press:** `drive_helpers.py` applies `set_speed_offset` only on long‑press events, with sign based on accel/decel and an adjustment when the offset would go negative.
- **Gas‑pressed clipping:** `drive_helpers.py` clamps `v_cruise` to at least `vEgo` when gas is pressed and the user decels/sets cruise, preventing a below‑current set speed while overriding.

## Key helpers, variables, and flows (what they do)
- **`VCruiseHelper` state:**
  - `v_cruise_kph` / `v_cruise_cluster_kph`: published to UI and downstream logic; must be stable under override.
  - `button_timers` + `button_change_states`: determine tap vs hold and guard against edge‑case double‑taps.
- **`CurveSpeedController` state:**
  - Curve target can reduce `v_cruise` and must continue to apply when the low‑speed override is active.
- **Button cadence:** `CRUISE_LONG_PRESS` defines the hold cadence threshold; `CRUISE_NEAREST_FUNC` handles rounding for ±5 logic.
- **SLC prompt suppression:** `frogpilotPlan.speedLimitChanged` is used in `controlsd` to avoid adjusting speed during confirm/deny.
- **Speed aggregation:** `FrogPilotVCruise.update(...)` combines curve speed and SLC with the set speed, enforcing SLC limits when enabled.

## Data flow (current)
1. **Buttons & PCM state → `VCruiseHelper`** (`drive_helpers.py`)
2. **`controlsState.vCruise` / `vCruiseCluster` → FrogPilotPlanner** (`frogpilot_planner.py`)
3. **FrogPilotVCruise** applies SLC / curve speed to set `frogpilotPlan.vCruise`.
4. **Longitudinal planner** consumes `frogpilotPlan.vCruise` to drive the car.
5. **UI** reads `controlsState` and renders MAX speed.

## Architecture clarification (Toyota with gas interceptor)

**Key flags for Toyota with pedal mod:**
- `pcmCruise = True` (default, never overridden for Toyota) — PCM **reports** set speed
- `openpilotLongitudinalControl = True` — OP **controls** actual gas/brake
- `enableGasInterceptor = True` — pedal mod installed

**What this means:**
- The PCM never actually controls gas/brake on a car with gas interceptor
- PCM only **reports** what it thinks the set speed should be via `CS.cruiseState.speed`
- OpenPilot reads that value and uses it for longitudinal planning
- OpenPilot executes gas/brake via the interceptor

**Implication for low-speed override:**
- We don't need to "send commands to the PCM" — PCM is passive
- We just need to change what `VCruiseHelper.v_cruise_kph` is set to
- The rest of the system (FrogPilot vcruise, SLC, longitudinal planner) just uses that value
- PCM continues reporting ~28 mph, but we ignore it

## Simplification opportunity (architectural insight)

**Current implementation is over-complicated.** The existing `_update_v_cruise_non_pcm()` already handles:
- Button tap vs hold detection
- Standstill checks
- Enabled state checks
- Speed limit changed checks
- Increment calculation with rounding
- Clipping

**The 250-line `toyota_low_speed_cruise.py` module duplicates most of this.** A simpler approach:

```python
# Pseudocode for minimal implementation
if at_pcm_floor and should_enter_low_speed_mode:
    self.v_cruise_kph = v_ego_kph  # Initialize to current speed
    self.low_speed_override_active = True

if self.low_speed_override_active:
    # Reuse existing button handling with correct increment
    self._update_v_cruise_non_pcm(CS, enabled, is_metric, ...)
    self.update_button_timers(CS, enabled)

    if self.v_cruise_kph >= V_CRUISE_PCM_FLOOR:
        self.low_speed_override_active = False  # Exit, hand back to PCM
else:
    self.v_cruise_kph = pcm_v_cruise_kph  # Normal: read from PCM
```

**Toggle compatibility issue:**
- `cruise_increase` / `cruise_increase_long` are only configured for non-PCM vehicles
- For PCM cruise, they fall back to defaults: tap=1, hold=5
- But Toyota PCM behavior should be:
  - `reverse_cruise_increase=False`: tap=1, hold=1
  - `reverse_cruise_increase=True`: tap=5, hold=5
- **Fix:** Override increment in low-speed mode based on `reverse_cruise_increase`

## Required behavior scenarios (must match FrogPilot semantics)
> The following are minimum acceptance behaviors. Ensure both **tap** and **hold** logic match stock FrogPilot cadence and rounding.

1. **Scenario 1 (tap down, ±5):** 37 → 35 → 30 → **override** to 25 → 20.
2. **Scenario 2 (hold down, ±5):** 37 → … → 20 with continuous adjustments, with override engaged seamlessly at floor.
3. **Scenario 3 (tap up, ±5):** 22 → 25 → 30 (override) → 35 (handoff back to PCM).
4. **Scenario 4 (hold up, ±5):** 22 → … → 35 with continuous adjustments and smooth handoff.
5. **Scenarios 5–8:** Same as 1–4 but **±1** (toggle off).

## Implementation guidance (extra scrutiny)
- **Button tap/hold correctness:** Use `update_button_timers(...)` and `CRUISE_LONG_PRESS` semantics; avoid edge‑detection regressions or multiple‑tap requirements.
- **Seamless transitions:** Ensure handoff between PCM and override does not add delays or change cadence.
- **No flicker:** While override is active, set `vCruise` and `vCruiseCluster` to the OP‑owned value to prevent UI flicker.
- **Respect SLC:** Effective target should still be clamped by SLC outputs when enabled.
- **Smallest footprint:** Prefer a dedicated helper and minimal wiring rather than touching unrelated modules.
- **Safety:** Changes must honor openpilot safety expectations and not bypass safety gating logic.

## Internet usage
Use the internet for research as needed (e.g., Toyota DSU/PCM behavior, FrogPilot community docs), but do not add conflicting info without verifying against the codebase and updating **both** documents.

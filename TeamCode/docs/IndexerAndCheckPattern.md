# Indexer and Check Pattern Subsystems

**Overview**
- **Purpose:** `IndexerSubsystem` precisely positions the indexer to intake and launch slots using encoder phases corrected by a hall-effect magnet sensor; `CheckPatternSubsystem` samples a color sensor at launch slots to classify each slot as "empty", "green", or "purple".
- **Integration:** `CheckPatternSubsystem` commands `IndexerSubsystem` to move to each launch slot, waits until motion settles, samples HSV, and records a 3-slot pattern.
- **Key Files:** [IndexerSubsystem.java](../src/main/java/org/firstinspires/ftc/teamcode/subsystems/IndexerSubsystem.java) and [CheckPatternSubsystem.java](../src/main/java/org/firstinspires/ftc/teamcode/subsystems/CheckPatternSubsystem.java).

**Indexer Subsystem**
- **Virtual Phase:** Computes a virtual phase `wrap(rawEncoder + phaseOffsetTicks, TICKS_PER_REV)` to eliminate accumulated drift.
- **Magnet Correction:** Uses a hall sensor pulse center (rise+fall average) to snap `phaseOffsetTicks` so virtual phase aligns to known launch magnet phases `MAG_LAUNCH_[1..3]_PHASE` within `MAGNET_WINDOW_TICKS`.
- **Slots:** Launch phases (`LAUNCH_1/2/3`) align to magnets; intake phases (`INTAKE_1/2/3`) lie between magnets and remain stable because magnet snaps keep the offset corrected.
- **Motion Control:** P-controller moves toward `targetPhase` on the virtual phase circle: $pwr = k_P \cdot err$, clamped to `maxPower` and with `minPower` applied only when `|err|` > `minPowerDisableWithinTicks`. Arrival uses `deadbandTicks` + `settleMs` before finishing.
- **Lever Pulse:** A feed lever servo pulses to `leverEngagedPos` for `leverPulseMs` when requested, then returns to `leverIdlePos`. `setLeverMax()` caps all positions by `leverMaxPos`.
- **Public API:**
  - **`update()`**: Runs lever pulse, magnet correction, motion controller, and debug sampling each loop.
  - **`setIndexerPosition(pos)`**: Accepts `"Launch1"|"Launch2"|"Launch3"|"Intake1"|"Intake2"|"Intake3"`, sets `targetPhase`.
  - **`nudgeTicks(delta)`**: Nudges `targetPhase` by encoder ticks (used by `CheckPatternSubsystem`).
  - **`isMoving()`**: Returns `true` while moving (manual mode disabled in current build).
  - **Telemetry:** `getStatus()` and debug getters (`getDbgVirtualPhase()`, `getDbgError()`, etc.).

**Magnet Correction Details**
- **Edge Handling:** On rising edge, record `risingEdgeRaw`; on falling edge, compute `centerRaw = round((rise+fall)/2)`.
- **Nearest Magnet:** Find nearest of `MAG_LAUNCH_1_PHASE`, `MAG_LAUNCH_2_PHASE`, `MAG_LAUNCH_3_PHASE` by shortest wrapped distance; reject if distance > `MAGNET_WINDOW_TICKS`.
- **Offset Snap:** Compute desired wrapped offset so `wrap(centerRaw + offset) == expectedMagPhase`. Update `phaseOffsetTicks` by shortest wrapped delta.
- **Debounce:** Require full pulse and `MAGNET_DEBOUNCE_MS` between snaps.

**Motion Controller Details**
- **Error:** `err = shortestSignedDelta(curVPhase, targetPhase, TICKS_PER_REV)`.
- **Power:** $pwr = k_P \cdot err$, with `|pwr| <= maxPower` and optional `minPower` when far from target.
- **Arrival:** If `|err| <= deadbandTicks`, set power to 0 and require `settleMs` continuous deadband before declaring `IDLE`.
- **Timeout:** Abort motion after `moveTimeoutMs` as a safety.

**Check Pattern Subsystem**
- **Goal:** Sample the color at each launch slot to classify the three-slot pattern: `[slot0, slot1, slot2]`.
- **State Machine:**
  - **`MOVE`**: Call `indexer.setIndexerPosition("LaunchX")` for current slot.
  - **`SETTLE`**: Wait until `indexer.isMoving()` is false and `settleMs` elapsed (or `nudgeSettleMs` after a black-nudge).
  - **`SAMPLE`**: Take `samplesPerRead` readings spaced by `sampleSpacingMs`; average HSV.
  - **`BLACK_NUDGE`**: If average looks black (hole/background), alternate `indexer.nudgeTicks(±nudgeTicks)` up to `maxBlackRetries` to disambiguate; then re-settle and re-sample.
  - **`DONE`**: After all three slots, stop checking.
- **Classification:**
  - **Black Test:** `avgVal <= blackValueMax` OR `avgSat <= blackSaturationMax` → `"empty"`.
  - **Hue Match:** Near `greenHue ± hueTol` → `"green"`; near `purpleHue ± purpleHueTol()` → `"purple"`.
  - **Else:** `"unknown"`.
- **Outputs:** `getIndexerPattern()` returns string array of length 3 with values `"empty"|"green"|"purple"|"unknown"`.

**Tuning Parameters**
- **Indexer:** `TICKS_PER_REV`, `MAG_LAUNCH_[1..3]_PHASE`, `LAUNCH_[1..3]`, `INTAKE_[1..3]`, `MAGNET_WINDOW_TICKS`, `MAGNET_DEBOUNCE_MS`, `kP`, `maxPower`, `minPower`, `deadbandTicks`, `settleMs`, `moveTimeoutMs`, `minPowerDisableWithinTicks`, lever positions and pulse timings.
- **CheckPattern:** `settleMs`, `samplesPerRead`, `sampleSpacingMs`, `greenHue`, `purpleHue`, `hueTol`, `blackValueMax`, `blackSaturationMax`, `maxBlackRetries`, `nudgeTicks`, `nudgeSettleMs`.

**Operational Flow**
- **TeleOp Loop:** Call `indexer.update()` every cycle. When running a pattern check, call `checkPattern.update()` every cycle until `getStatus()` becomes false.
- **Competition Use:** Keep `MAGNET_CORRECTION_ENABLED = true` in the indexer to ensure reliable phase alignment. Manual raw-power tuning is currently disabled in code.

**Troubleshooting**
- **Jitter near target:** Increase `deadbandTicks` or decrease `kP`; consider increasing `minPowerDisableWithinTicks`.
- **Missed magnet snaps:** Widen `MAGNET_WINDOW_TICKS` slightly or verify `TICKS_PER_REV` and magnet phase constants.
- **Slow settling:** Increase `settleMs` (indexer) or `nudgeSettleMs` (check pattern) depending on which state is slow.
- **Color misclassification:** Adjust `greenHue`, `purpleHue`, and `hueTol`; verify sensor distance and lighting; tune black thresholds.

**Usage Examples**
- **Start check pattern:**
  - Create once: `CheckPatternSubsystem cps = new CheckPatternSubsystem(hardwareMap, indexer, "colorSensor");`
  - Start: `cps.start();`
  - In loop: `cps.update();`
  - After done: `String[] pattern = cps.getIndexerPattern();`
- **Move indexer directly:**
  - `indexer.setIndexerPosition("Intake1");` then in loop `indexer.update();` until `!indexer.isMoving()`.

See sources for exact field names and current defaults: [IndexerSubsystem.java](../src/main/java/org/firstinspires/ftc/teamcode/subsystems/IndexerSubsystem.java), [CheckPatternSubsystem.java](../src/main/java/org/firstinspires/ftc/teamcode/subsystems/CheckPatternSubsystem.java).

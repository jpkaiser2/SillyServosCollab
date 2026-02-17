# Robot Controls

This document describes the teleop controls used by `TeleOpMainPedroTemplate`.

## Gamepad 1 — Driver

- Drive:
  - Left stick (Y): forward/backward
  - Left stick (X): strafe left/right
  - Right stick (X): rotate
- Intake (roller):
  - Left bumper: intake forward (pull game pieces in)
  - Left trigger: intake reverse (push game pieces out)
  - Threshold: triggers/bumpers engage above ~0.4
- Intake angle (arm rotation):
  - Right trigger: rotate down
  - Right bumper: rotate up
- Ramp (normal mode):
  - Left bumper/Left trigger: ramp down (unless intake is holding up)
  - Otherwise: ramp up
- Indexer presets (intake positions):
  - D-pad Left: `Intake1`
  - D-pad Down: `Intake2`
  - D-pad Right: `Intake3`

## Gamepad 2 — Operator

- Override mode:
  - Hold Left trigger (> ~0.4) to enter override.
- Turret + Flywheel (override):
  - Right stick (X): turret yaw
  - Left stick (Y): turret angle
  - Right bumper / Right trigger: set flywheel to 96 RPM
- Turret (normal mode):
  - Right bumper / Right trigger: request auto launching/aiming
- Launch sequence:
  - `Y`: start auto launch (if not already launching)
  - Override + Right trigger: stop auto launch (when launching)
- Pattern scan sequence:
  - `X` or `A` or `B`: start pattern scan (if not launching)
  - Override + Right trigger: stop pattern scan (when scanning)
- Set wanted pattern:
  - `X`: [green, purple, purple]
  - `A`: [purple, green, purple]
  - `B`: [purple, purple, green]
- Indexer presets (launch positions):
  - D-pad Left: `Launch1`
  - D-pad Down: `Launch2`
  - D-pad Right: `Launch3`
- Feed lever:
  - D-pad Up: pulse feed lever (disabled during auto modes)
- Ramp (override):
  - Any of: Gamepad 2 Left bumper OR Gamepad 1 Left bumper OR Gamepad 1 Left trigger → ramp down
  - Otherwise: ramp up

## Notes

- Intake hold: Certain indexer actions temporarily hold the intake angle up; it auto-releases when indexer movement finishes.
- `triggerSense` threshold is ~0.4 for analog triggers.
- Telemetry reports key subsystem statuses during operation.

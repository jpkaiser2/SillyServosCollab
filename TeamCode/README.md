# TeamCode: Robot Architecture and TeleOp Guide

This document explains how the robot code is organized, how each subsystem works, why certain strategies were chosen, and how to operate and extend the system. The intent is to make the codebase approachable for new team members and easy to maintain throughout the season.

## High-Level Architecture

- "Subsystem" pattern: Each mechanism is encapsulated in its own class with a small API (`update()`, setters for inputs, and `getStatus()` for telemetry). Subsystems are non-blocking and avoid `sleep()`; any timed actions use `ElapsedTime` so the main loop stays responsive.
- Drive abstraction: `DriveBase` is an interface for the drivetrain. The TeleOp talks to the interface, not the concrete drive. We currently run a raw mecanum implementation (`RawMecanumDrive`) and can later swap to a pathing drive (e.g., Pedro Pathing) by implementing the same interface.
- TeleOp orchestration: `TeleOpPedroTemplate` wires gamepad inputs to subsystems, calls `update()` each loop, and provides telemetry for drivers and debugging.

```
TeleOpPedroTemplate
  ├─ DriveBase (RawMecanumDrive now, Pedro later)
  ├─ TurretSubsystem (motor + angle servo)
  ├─ IntakeSubsystem (motor + angle servo)
  ├─ IndexerSubsystem (motor with encoder + feed lever servo)
  └─ FlywheelSubsystem (motor)
```

## Drivetrain

- File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/drive/RawMecanumDrive.java`
- Why: Raw mecanum math is simple, responsive, and reliable for TeleOp. We avoid field-centric for simplicity; inputs are robot-centric.
- Inputs (Gamepad1):
  - `left_stick_y` (forward/back)
  - `left_stick_x` (strafe)
  - `right_stick_x` (rotate)
  - `left_bumper` (hold) → slow mode (~40%) for precision
- Strategy: Normalize wheel powers to avoid clipping, keep the loop non-blocking, and keep IMU out of the loop for predictability.
- Future: To swap in Pedro Pathing, implement a `PedroDrive implements DriveBase` and replace the constructor line in TeleOp. All other code stays unchanged.

## Turret Subsystem

- File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/TurretSubsystem.java`
- Hardware: `DcMotorEx` (rotation) + `Servo` (angle).
- Inputs (Gamepad2):
  - `right_stick_x` → turret rotation (motor power)
  - `left_stick_y` → turret angle (servo position)
- Strategy: Keep rotation responsive with BRAKE zero power behavior; angle servo uses a simple mapped input and is reasserted each loop to resist drift.

## Intake Subsystem

- File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IntakeSubsystem.java`
- Hardware: Core Hex motor (intake roller) + `Servo` (intake angle/arm).
- Inputs (Gamepad2):
  - `right_trigger` → intake in (forward)
  - `left_trigger` → intake out (reverse)
  - `left_stick_x` → intake angle servo adjustment
- Strategy: Triggers make motor direction intuitive; servo angle uses a small deadband and continuous reassertion to “hold” against bumps.

## Indexer Subsystem

- File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IndexerSubsystem.java`
- Hardware: `DcMotorEx` with encoder (indexer rotation) + `Servo` (feed lever).
- Behavior:
  - Preset-only moves: The indexer moves to predefined encoder targets using `RUN_TO_POSITION` with moderate power for smoothness.
  - Feed lever: Short non-blocking pulse controlled by a timer (no sleeps) to actuate feeding without blocking the main loop.
- Primary presets (encoder ticks):
  - `POSITION_1 = 3`
  - `POSITION_2 = 97`
  - `POSITION_3 = 190`
- Collection presets (encoder ticks):
  - `COLLECTION_1 = 332`
  - `COLLECTION_2 = 241`
  - `COLLECTION_3 = 429`
- Inputs (Gamepad2):
  - D-Pad Up/Right/Down → primary presets 1/2/3
  - Hold `left_bumper` + D-Pad Up/Right/Down → collection presets 1/2/3
  - `Y` → pulse the feed lever
- Notes:
  - The indexer motor uses BRAKE zero power behavior and `RUN_TO_POSITION` for deterministic moves.
  - Manual tuning mode was intentionally removed for competition readiness. Presets can be adjusted in code before a match if needed.
  - Collection modifier uses `left_bumper` on Gamepad2 to avoid conflicts with intake reverse.

## Flywheel Subsystem

- File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/FlywheelSubsystem.java`
- Hardware: `DcMotorEx`.
- Input (Gamepad2): `right_bumper` (hold) → run flywheel at full power
- Strategy: Simple “hold to run” avoids state drift and makes intent obvious.

## TeleOp Orchestration

- File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/TeleOpPedroTemplate.java`
- Responsibilities:
  - Read gamepad inputs, map them to subsystem commands
  - Call `update()` on subsystems each loop
  - Emit concise telemetry for driver awareness
- Telemetry includes: drive inputs, turret and intake status, indexer status (selection, busy flag, lever pulse state), flywheel status.

## Non-Blocking Timing and Safety

- No `Thread.sleep()`: Time-based actions (e.g., feed lever pulse) use `ElapsedTime` and are handled inside `update()` so the loop remains responsive.
- BRAKE behaviors: Motors that hold position (turret, indexer) use BRAKE zero power behavior to resist back-driving.
- Normalization and deadbands: Stick deadbands and normalized power prevent jitter and ensure smooth motion.

## Hardware Map Names

Ensure your configuration names match TeleOp initialization:

- Drive motors: `frontLeft`, `frontRight`, `backLeft`, `backRight`
- Turret motor: `turret`
- Turret angle servo: `turretAngle`
- Intake motor: `intake`
- Intake angle servo: `intakeAngle`
- Indexer motor: `indexer`
- Feed lever servo: `feedLever`
- Flywheel motor: `flywheel`

## Why These Strategies

- Subsystems + non-blocking loops: Keeps TeleOp responsive and code modular. It’s simpler to debug and safer under match conditions.
- Robot-centric drive: Reduces cognitive load for drivers and eliminates IMU drift dependencies during TeleOp.
- Encoder-based indexer presets: More repeatable than open-loop timing and easier to maintain than continuous tuning during matches.
- Minimal state toggles: Most actions are “hold to act,” making intent clear and avoiding unexpected latched states mid-match.

## Extending the System

- Swap in pathing: Implement `DriveBase` for Pedro Pathing (or other planners). Replace `RawMecanumDrive` with your new drive in TeleOp `init()`.
- Add autos: Reuse the same subsystems in OpModes for autonomous; keep `update()` non-blocking for consistent behavior.
- Adjust presets: Update the encoder constants in `IndexerSubsystem` for new mechanisms or calibration.
- Additional safety: Add soft limits to indexer ticks (min/max) if your mechanism risks hard stops.

## Build, Deploy, and Run

- Open the project in Android Studio or VS Code with FTC plugins.
- Connect to the RC, select the `Pedro Template` TeleOp.
- Verify hardware mapping matches your configuration.
- Use the control map above; confirm each subsystem responds as expected.

## Quick Reference: Controls

- Gamepad1 (Driver)
  - `LS Y` = forward/back, `LS X` = strafe, `RS X` = rotate
  - `LB` = slow mode
- Gamepad2 (Operator)
  - Turret: `RS X` rotate, `LS Y` angle
  - Intake: `RT` in, `LT` out, `LS X` angle
  - Indexer: `Y` feed lever pulse
  - Presets: D-Pad (primary), hold `LB` + D-Pad (collection)
  - Flywheel: hold `RB`

---


## Autonomous Path State Machine

- File: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/AutoPathStateMachine.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/AutoPathStateMachine.java)
- Purpose: Sequence any number of Pedro `PathChain`s with a `Follower` using a simple `start()` + `update()` loop.
- Usage:

```java
// In your OpMode
AutoPathStateMachine auto = new AutoPathStateMachine(follower)
  .add(pathA)
  .add(pathB)
  .add(pathC);

// In loop()
if (auto.getState() == AutoPathStateMachine.State.IDLE) auto.start();
auto.update(); // moves to next path when the follower finishes

// Telemetry (optional)
telemetry.addData("Auto State", auto.getState());
telemetry.addData("Auto Index", auto.getIndex());
```



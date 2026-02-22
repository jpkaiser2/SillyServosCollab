package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.configurables.PanelsConfigurables;

import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveBase;
import org.firstinspires.ftc.teamcode.subsystems.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.CheckPatternSubsystem;

@TeleOp(name = "TeleOpMainPedro", group = "TeleOp")
public class TeleOpMainPedroTemplate extends OpMode {

    // HardwareMap names
    private static final String FRONT_LEFT = "frontLeft";
    private static final String FRONT_RIGHT = "frontRight";
    private static final String BACK_LEFT = "backLeft";
    private static final String BACK_RIGHT = "backRight";
    private static final String TURRET = "turret";
    private static final String TURRET_ANGLE = "turretAngle";
    private static final String INTAKE = "intake";
    private static final String INTAKE_ANGLE = "intakeAngle";
    private static final String FEED_LEVER = "feedLever";
    private static final String INDEXER = "indexer";
    private static final String FLYWHEEL = "flywheel";
    private static final String RAMP = "ramp";
    private static final String COLOR_SENSOR = "sensor_color";
    private static final String IMU = "imu";

    private DriveBase drive;
    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    private FlywheelSubsystem flywheel;
    private LaunchSubsystem launch;
    private CheckPatternSubsystem pattern;

    // Indexer preset control
    private String prevIndex = "";
    private boolean intakeHold = false;

    // bumpers -> doubles
    private double p1LeftBumperToDouble;
    private double p1RightBumperToDouble;
    private double p2LeftBumperToDouble;
    private double p2RightBumperToDouble;

    private final double triggerSense = 0.4;

    // Override mode
    private boolean override = false;

    // Flywheel
    private double flywheelSpeed = 96;
    private boolean launching = false;

    // Patterns
    private String[] wantedPattern = { "purple", "green", "purple" };
    private String[] indexerPattern = { "empty", "empty", "empty" };
    private boolean checkingPattern = false;
    private boolean autoLaunching = false;

    // Indexer tuning / manual mode
    private boolean manualIndexerMode = false;
    private boolean prevManualToggle = false;
    private double manualIndexerPower = 0.22;
    private double manualDeadband = 0.15;

    // Indexer nudge
    private int nudgeTickAmount = 8;
    private boolean prevNudgeForward = false;
    private boolean prevNudgeBackward = false;

    // Hood positions for close/far launch
    private double hoodClosePosition = 0.7;  // Hood up for close shots
    private double hoodFarPosition = 0.2;    // Hood down for far shots
    private boolean prevHoodClose = false;
    private boolean prevHoodFar = false;

    @Override
    public void init() {
        HardwareMap hw = hardwareMap;

        drive = new PedroDrive(hw);
        intake = new IntakeSubsystem(hw, INTAKE, INTAKE_ANGLE, RAMP);
        indexer = new IndexerSubsystem(hw, INDEXER, FEED_LEVER);
        flywheel = new FlywheelSubsystem(hw, FLYWHEEL);
        turret = new TurretSubsystem(hw, TURRET, TURRET_ANGLE, telemetry, flywheel);
        launch = new LaunchSubsystem(indexer);
        pattern = new CheckPatternSubsystem(hw, indexer, COLOR_SENSOR);

        try {
            PanelsConfigurables.INSTANCE.refreshClass(indexer);
        } catch (Exception ignore) {
        }
    }

    @Override
    public void loop() {
        // Convert bumpers to doubles
        p1LeftBumperToDouble = gamepad1.left_bumper ? 1.0 : 0.0;
        p1RightBumperToDouble = gamepad1.right_bumper ? 1.0 : 0.0;
        p2LeftBumperToDouble = gamepad2.left_bumper ? 1.0 : 0.0;
        p2RightBumperToDouble = gamepad2.right_bumper ? 1.0 : 0.0;

        // FORCE OVERRIDE MODE (AUTO LAUNCHING DISABLED)
        // override = gamepad2.left_trigger > triggerSense;
        override = true;

        // Drive
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        drive.setDriverInput(x, y, rx, true);
        drive.update();

        // Intake
        intake.setTriggers(p1LeftBumperToDouble, gamepad1.left_trigger);

        if (!intakeHold) {
            if (gamepad1.right_trigger > triggerSense) {
                intake.setRotationInput(-0.5);
            } else if (gamepad1.right_bumper) {
                intake.setRotationInput(0.35);
            }
        }

        // ==========================================================
        // INDEXER MANUAL TUNING MODE (TOGGLE + HOLD-TO-SPIN)
        // ==========================================================
        boolean togglePressed = gamepad2.back;
        if (togglePressed && !prevManualToggle) {
            manualIndexerMode = !manualIndexerMode;
        }
        prevManualToggle = togglePressed;

        /*if (manualIndexerMode) {
            double stick = -gamepad2.left_stick_y; // up positive

            if (stick > manualDeadband) {
                indexer.setManualPower(manualIndexerPower);
            } else if (stick < -manualDeadband) {
                indexer.setManualPower(-manualIndexerPower);
            } else {
                indexer.stopManual();
            }

            // Optional: still allow lever pulse
            indexer.handleLeverButton(gamepad2.dpad_up);

            // Update indexer so magnet snapping + debug works
            indexer.update();

            telemetry.addLine("=== INDEXER MANUAL TUNING MODE ===");
            telemetry.addLine("Toggle: gamepad2 BACK");
            telemetry.addLine("Spin: gamepad2 left stick Y");
            telemetry.addData("indexerPosition", indexer.getCurrentPosition());
            telemetry.addData("MagPressed", indexer.getMagnetState());
            telemetry.addData("RawPhase", indexer.getDbgRawPhase());
            telemetry.addData("VirtualPhase", indexer.getDbgVirtualPhase());
            telemetry.addData("Offset", indexer.getDbgPhaseOffsetTicks());
            telemetry.addData("LastMag", indexer.getDbgLastMagnetName());
            telemetry.addData("LastSnap", indexer.getDbgLastSnapError());
            telemetry.addData("PhaseErr", indexer.getDbgError());

            // New 3-magnet debug (pulse center capture)
            telemetry.addData("RiseRaw", indexer.getDbgRiseRaw());
            telemetry.addData("FallRaw", indexer.getDbgFallRaw());
            telemetry.addData("CenterRaw", indexer.getDbgCenterRaw());
            telemetry.addData("LastMagPhase", indexer.getDbgLastMagnetPhase());

            telemetry.update();
            return;
        }*/

        // ----------------------------
        // Indexer Nudge (gamepad1.a = forward, gamepad1.b = backward)
        // ----------------------------
        boolean nudgeForward = gamepad2.x;
        boolean nudgeBackward = gamepad2.b;

        if (nudgeForward && !prevNudgeForward) {
            indexer.nudgeTicks(nudgeTickAmount);
        }
        prevNudgeForward = nudgeForward;

        if (nudgeBackward && !prevNudgeBackward) {
            indexer.nudgeTicks(-nudgeTickAmount);
        }
        prevNudgeBackward = nudgeBackward;

        // ----------------------------
        // Manual Indexer Presets (ALWAYS ENABLED - AUTO LAUNCHING DISABLED)
        // ----------------------------
        // if (!(checkingPattern || autoLaunching)) {
        if (true) {
            if (gamepad1.dpad_left && !"P1Left".equals(prevIndex)) {
                indexer.setIndexerPosition("Intake1");
                intakeHold = true;
                intake.setHoldUp(true);
            } else if (gamepad1.dpad_down && !"P1Down".equals(prevIndex)) {
                indexer.setIndexerPosition("Intake2");
                intakeHold = true;
                intake.setHoldUp(true);
            } else if (gamepad1.dpad_right && !"P1Right".equals(prevIndex)) {
                indexer.setIndexerPosition("Intake3");
                intakeHold = true;
                intake.setHoldUp(true);
            } else if (gamepad2.dpad_left && !"P2Left".equals(prevIndex)) {
                indexer.setIndexerPosition("Launch1");
                intakeHold = true;
                intake.setHoldUp(true);
            } else if (gamepad2.dpad_down && !"P2Down".equals(prevIndex)) {
                indexer.setIndexerPosition("Launch2");
                intakeHold = true;
                intake.setHoldUp(true);
            } else if (gamepad2.dpad_right && !"P2Right".equals(prevIndex)) {
                indexer.setIndexerPosition("Launch3");
                intakeHold = true;
                intake.setHoldUp(true);
            }

            indexer.handleLeverButton(gamepad2.dpad_up);
        } else {
            indexer.handleLeverButton(false);
        }

        // Hood control for close/far launch
        boolean hoodCloseTrigger = gamepad2.left_stick_button;
        boolean hoodFarTrigger = gamepad2.right_stick_button;

        if (hoodCloseTrigger && !prevHoodClose) {
            turret.setHoodPosition(hoodClosePosition);
        }
        prevHoodClose = hoodCloseTrigger;

        if (hoodFarTrigger && !prevHoodFar) {
            turret.setHoodPosition(hoodFarPosition);
        }
        prevHoodFar = hoodFarTrigger;

        // Override / Auto logic
        if (override) {
            if (gamepad2.left_bumper || gamepad1.left_bumper || gamepad1.left_trigger > triggerSense) {
                intake.setRampDown();
            } else {
                intake.setRampUp();
            }

            if (gamepad2.right_bumper || gamepad2.right_trigger > triggerSense) {
                flywheelSpeed = 96;
            } else {
                flywheelSpeed = 0;
            }

            turret.updateManual(gamepad2.right_stick_x, gamepad2.left_stick_y, flywheelSpeed);

        }
        
        /* AUTO LAUNCHING DISABLED - ALWAYS IN OVERRIDE MODE
        } else {
            if ((gamepad1.left_bumper || gamepad1.left_trigger > triggerSense) && !intakeHold) {
                intake.setRampDown();
            } else {
                intake.setRampUp();
            }

            launching = (gamepad2.right_bumper || gamepad2.right_trigger > triggerSense);
            turret.updateAutoTurret(launching);
        }
        */

        /* AUTO LAUNCHING DISABLED
        // Launch sequence (Y)
        if (gamepad2.y && !checkingPattern) {
            if (override && gamepad2.right_trigger > triggerSense && autoLaunching) {
                launch.stopLaunch();
            } else if (!autoLaunching) {
                launch.startLaunch(wantedPattern, indexerPattern);
            }
        }
        // Pattern scan (X/A/B)
        else if ((gamepad2.x || gamepad2.a || gamepad2.b) && !autoLaunching) {
            if (override && gamepad2.right_trigger > triggerSense && checkingPattern) {
                pattern.stop();
            } else if (!checkingPattern) {
                pattern.start();
            }
        }
        */

        // Set wanted pattern
        if (gamepad2.x) {
            wantedPattern = new String[] { "green", "purple", "purple" };
        } else if (gamepad2.a) {
            wantedPattern = new String[] { "purple", "green", "purple" };
        } else if (gamepad2.b) {
            wantedPattern = new String[] { "purple", "purple", "green" };
        }

        // Subsystem updates
        indexer.update();
        /* AUTO LAUNCHING DISABLED
        launch.update();
        pattern.update();

        autoLaunching = launch.getStatus();
        checkingPattern = pattern.getStatus();

        if (autoLaunching) {
            indexerPattern = launch.getIndexerPattern();
        } else if (checkingPattern) {
            indexerPattern = pattern.getIndexerPattern();
        }
        */

        if (!indexer.isMoving()) {
            intakeHold = false;
            intake.setHoldUp(false);
        }

        // prevIndex edge tracking
        if (gamepad1.dpad_left)
            prevIndex = "P1Left";
        else if (gamepad1.dpad_down)
            prevIndex = "P1Down";
        else if (gamepad1.dpad_right)
            prevIndex = "P1Right";
        else if (gamepad2.dpad_left)
            prevIndex = "P2Left";
        else if (gamepad2.dpad_down)
            prevIndex = "P2Down";
        else if (gamepad2.dpad_right)
            prevIndex = "P2Right";
        else
            prevIndex = "";

        // Telemetry (full)
        telemetry.addData("currentHue", pattern.getHsv()[0]);
        telemetry.addData("overrideState", override);
        telemetry.addData("magnetState", indexer.getMagnetState());
        telemetry.addData("indexerPosition", indexer.getCurrentPosition());
        telemetry.addData("wantedPattern", printStringArray(wantedPattern));
        // telemetry.addData("currentPattern", printStringArray(indexerPattern));
        // telemetry.addData("launching(auto)", autoLaunching);
        // telemetry.addData("checkingPattern", checkingPattern);
        telemetry.addData("Drive", "x=%.2f y=%.2f rx=%.2f", x, y, rx);
        telemetry.addData("Turret", turret.getStatus());
        telemetry.addData("Intake", intake.getStatus());
        telemetry.addData("IntakeHold", intakeHold);
        telemetry.addData("IntakeHoldUp", intake.isHoldUp());
        telemetry.addData("Indexer", indexer.getStatus());
        telemetry.addData("Flywheel", flywheel.getStatus());

        // Extra tuning info (phase + magnets)
        telemetry.addData("RawPhase", indexer.getDbgRawPhase());
        telemetry.addData("VirtualPhase", indexer.getDbgVirtualPhase());
        telemetry.addData("Offset", indexer.getDbgPhaseOffsetTicks());
        telemetry.addData("LastMag", indexer.getDbgLastMagnetName());
        telemetry.addData("LastSnap", indexer.getDbgLastSnapError());
        telemetry.addData("PhaseErr", indexer.getDbgError());

        // 3-magnet pulse debug
        telemetry.addData("RiseRaw", indexer.getDbgRiseRaw());
        telemetry.addData("FallRaw", indexer.getDbgFallRaw());
        telemetry.addData("CenterRaw", indexer.getDbgCenterRaw());
        telemetry.addData("LastMagPhase", indexer.getDbgLastMagnetPhase());

        telemetry.update();
    }

    // Print arrays
    public String printStringArray(String[] arr) {
        String printThis = "[";
        for (int i = 0; i < arr.length; i++) {
            printThis += arr[i];
            if (i < arr.length - 1)
                printThis += ", ";
        }
        printThis += "]";
        return printThis;
    }
}

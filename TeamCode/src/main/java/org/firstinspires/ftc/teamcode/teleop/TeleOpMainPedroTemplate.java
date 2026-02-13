package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    // HardwareMap names (edit these to match your configuration)
    private static final String FRONT_LEFT = "frontLeft";
    private static final String FRONT_RIGHT = "frontRight";
    private static final String BACK_LEFT = "backLeft";
    private static final String BACK_RIGHT = "backRight";
    private static final String TURRET = "turret";            // motor
    private static final String TURRET_ANGLE = "turretAngle";  // servo
    private static final String INTAKE = "intake";             // core hex motor
    private static final String INTAKE_ANGLE = "intakeAngle";  // servo
    private static final String FEED_LEVER = "feedLever";      // servo
    private static final String INDEXER = "indexer";           // motor
    private static final String FLYWHEEL = "flywheel";         // motor
    private static final String RAMP = "ramp";                 // servo
    private static final String COLOR_SENSOR = "sensor_color"; // color sensor
    private static final String IMU = "imu";                   // optional

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

    private boolean override = false;
    private ElapsedTime overrideTime = new ElapsedTime();
    // Track press state and per-hold toggle count to avoid unintended flips on release
    private boolean overrideWasPressed = false;
    private int overrideToggleCount = 0;

    // fixing ftc's very annoying code lol(bumpers are bools, triggers are doubles)
    private double p1LeftBumperToDouble;
    private double p1RightBumperToDouble;
    private double p2LeftBumperToDouble;
    private double p2RightBumperToDouble;
    private final double triggerSense = 0.4;

    // Target flywheel speed in RPM (96 rpm ≈ 1.6 rps)
    private double flywheelSpeed = 96;
    private boolean launching = false;

    // Default values
    // wantedPattern is the pattern for the next 3 balls
    // indexerPattern is what is in the indexer
    // possible values for indexerPattern: empty, purple, green, unknown
    private String[] wantedPattern = {"purple", "green", "purple"};
    private String[] indexerPattern = {"empty", "empty", "empty"};
    private boolean checkingPattern = false;
    private boolean autoLaunching = false;

    @Override
    public void init() {
        HardwareMap hw = hardwareMap;

        // Use Pedro Pathing Follower for teleop drive
        drive = new PedroDrive(hw);
        intake = new IntakeSubsystem(hw, INTAKE, INTAKE_ANGLE, RAMP);
        indexer = new IndexerSubsystem(hw, INDEXER, FEED_LEVER);
        flywheel = new FlywheelSubsystem(hw, FLYWHEEL);
        turret = new TurretSubsystem(hw, TURRET, TURRET_ANGLE, telemetry, flywheel);
        launch = new LaunchSubsystem(indexer);
        pattern = new CheckPatternSubsystem(hw, indexer, COLOR_SENSOR);

        // Enable dashboard configurables for indexer presets
        try { PanelsConfigurables.INSTANCE.refreshClass(indexer); } catch (Exception ignore) {}
    }

    @Override
    public void loop() {
        // fixing ftc's bad coding(bumpers are bools, triggers are doubles), making usable doubles for bumpers
        // if you need to input a bumper into a function expecting a trigger(and thus a double), it will not work
        if (gamepad1.left_bumper)
            p1LeftBumperToDouble = 1.0;
        else
            p1LeftBumperToDouble = 0.0;
        if (gamepad1.right_bumper)
            p1RightBumperToDouble = 1.0;
        else
            p1RightBumperToDouble = 0.0;
        if (gamepad2.left_bumper)
            p2LeftBumperToDouble = 1.0;
        else
            p2LeftBumperToDouble = 0.0;
        if (gamepad2.right_bumper)
            p2RightBumperToDouble = 1.0;
        else
            p2RightBumperToDouble = 0.0;

        override = gamepad2.left_trigger > triggerSense;

        // Drive inputs (FTC sticks: up is -y)
        double y  = -gamepad1.left_stick_y;   // forward
        double x  =  gamepad1.left_stick_x;   // strafe
        double rx =  gamepad1.right_stick_x;  // rotation

        drive.setDriverInput(x, y, rx, true);
        drive.update();

        // Intaking
        intake.setTriggers(p1LeftBumperToDouble, gamepad1.left_trigger);
        if (!intakeHold) {
            if (gamepad1.right_trigger > triggerSense) {
                intake.setRotationInput(-0.5);
            } else if (gamepad1.right_bumper) {
                intake.setRotationInput(0.5);
            }
        }

        // ----------------------------
        // Manual Indexer Presets (disabled during auto functions)
        // ----------------------------
        if (!(checkingPattern || autoLaunching)) {
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

            // Pulse launch arm (only when not in auto modes)
            indexer.handleLeverButton(gamepad2.dpad_up);
        } else {
            // If we are doing auto functions, don't pulse lever arm
            indexer.handleLeverButton(false);
        }

        // ----------------------------
        // Override / Auto logic
        // ----------------------------
        if (override) {
            // Ramp control: up/down only
            if (gamepad2.left_bumper || gamepad1.left_bumper || gamepad1.left_trigger > triggerSense) {
                intake.setRampDown();
            } else {
                intake.setRampUp();
            }

            // Override flywheel controls (placeholder)
            if (gamepad2.right_bumper || gamepad2.right_trigger > triggerSense) {
                flywheelSpeed = 96; // 96 rpm
            } else {
                flywheelSpeed = 0;
            }

            turret.updateManual(gamepad2.right_stick_x, gamepad2.left_stick_y, flywheelSpeed);

        } else {
            // Auto ramp control
            if ((gamepad1.left_bumper || gamepad1.left_trigger > triggerSense) && !intakeHold) {
                intake.setRampDown();
            } else {
                intake.setRampUp();
            }

            // Auto turret launch request
            if (gamepad2.right_bumper || gamepad2.right_trigger > triggerSense) {
                launching = true;
            } else {
                launching = false;
            }
            turret.updateAutoTurret(launching);
        }

        // ----------------------------
        // Launch sequence (Y)
        // ----------------------------
        if (gamepad2.y && !checkingPattern) {
            if (override && gamepad2.right_trigger > triggerSense && autoLaunching) {
                launch.stopLaunch();
            } else if (!autoLaunching) {
                launch.startLaunch(wantedPattern, indexerPattern);
            }
        }
        // ----------------------------
        // Pattern scan sequence (X/A/B)
        // ----------------------------
        else if ((gamepad2.x || gamepad2.a || gamepad2.b) && !autoLaunching) {
            if (override && gamepad2.right_trigger > triggerSense && checkingPattern) {
                pattern.stop();
            } else if (!checkingPattern) {
                pattern.start();
            }
        }

        // Set wanted pattern
        if (gamepad2.x) {
            wantedPattern = new String[]{"green", "purple", "purple"};
        } else if (gamepad2.a) {
            wantedPattern = new String[]{"purple", "green", "purple"};
        } else if (gamepad2.b) {
            wantedPattern = new String[]{"purple", "purple", "green"};
        }

        // ----------------------------
        // Subsystem updates
        // ----------------------------
        indexer.update();   // magnet update should be disabled inside IndexerSubsystem for now
        launch.update();
        pattern.update();

        // IMPORTANT: Update status booleans AFTER subsystem updates
        autoLaunching = launch.getStatus();
        checkingPattern = pattern.getStatus();

        // IMPORTANT: Update indexerPattern AFTER status booleans are refreshed
        if (autoLaunching) {
            indexerPattern = launch.getIndexerPattern();
        } else if (checkingPattern) {
            indexerPattern = pattern.getIndexerPattern();
        }

        // Normal auto-release of intake hold when indexer finishes
        if (!indexer.isMoving()) {
            intakeHold = false;
            intake.setHoldUp(false);
        }

        // define previous indexer input (clear when no dpad is pressed to allow repeat presses)
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
            prevIndex = ""; // allow re-press once released

        // Telemetry
        telemetry.addData("currentHue", pattern.getHsv()[0]);
        telemetry.addData("overrideState", override);
        telemetry.addData("magnetState", indexer.getMagnetState());
        telemetry.addData("indexerPosition", indexer.getCurrentPosition());
        telemetry.addData("wantedPattern", printStringArray(wantedPattern));
        telemetry.addData("currentPattern", printStringArray(indexerPattern));
        telemetry.addData("launching(auto)", autoLaunching);
        telemetry.addData("checkingPattern", checkingPattern);
        telemetry.addData("Drive", "x=%.2f y=%.2f rx=%.2f", x, y, rx);
        telemetry.addData("Turret", turret.getStatus());
        telemetry.addData("Intake", intake.getStatus());
        telemetry.addData("IntakeHold", intakeHold);
        telemetry.addData("IntakeHoldUp", intake.isHoldUp());
        telemetry.addData("Indexer", indexer.getStatus());
        telemetry.addData("Flywheel", flywheel.getStatus());
        telemetry.update();
    }

    // lets us output an array
    public String printStringArray(String[] arr) {
        String printThis = "[";
        for (int i = 0; i < arr.length; i++) {
            printThis += arr[i];
            if (i < arr.length - 1) printThis += ", ";
        }
        printThis += "]";
        return printThis;
    }
}

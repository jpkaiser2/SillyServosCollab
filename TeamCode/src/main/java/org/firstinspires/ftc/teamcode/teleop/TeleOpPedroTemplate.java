package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;
*/

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveBase;
import org.firstinspires.ftc.teamcode.subsystems.drive.PedroDrive;

/**
 * TeleOpPedroTemplate
 *
 * How to swap RawMecanumDrive to Pedro later:
 * - Create a class PedroDrive implements DriveBase with the same methods.
 * - In init(), replace `drive = new RawMecanumDrive(...)` with `drive = new PedroDrive(...)`.
 * - Keep calls to drive.setDriverInput(...) and drive.update() the same.
 */
@TeleOp(name = "TeleOpMainPedro", group = "TeleOp")
public class MyTerribleCode extends OpMode {

    // HardwareMap names (edit these to match your configuration)
    private static final String FRONT_LEFT = "frontLeft";
    private static final String FRONT_RIGHT = "frontRight";
    private static final String BACK_LEFT = "backLeft";
    private static final String BACK_RIGHT = "backRight";
    private static final String TURRET = "turret";           // motor
    private static final String TURRET_ANGLE = "turretAngle"; // servo
    private static final String INTAKE = "intake";            // core hex motor
    private static final String INTAKE_ANGLE = "intakeAngle"; // servo
    private static final String FEED_LEVER = "feedLever";     // servo
    private static final String INDEXER = "indexer";          // motor
    private static final String FLYWHEEL = "flywheel";        // motor
    private static final String RAMP = "ramp"; // servo
    private static final String IMU = "imu"; // optional
    // private static final String COLOR_SENSOR = "sensor_color"; // color sensor at shooting position

    private DriveBase drive;
    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    private FlywheelSubsystem flywheel;
    // Indexer preset control
    
    private String prevIndex = "";
    private boolean intakeHold = false;
    private boolean override = false;
    private ElapsedTime overrideTime = new ElapsedTime();

    private double hoodInput = 0.0;
    private double turretInput = 0.0;

    // fixing ftc's very annoying code lol(bumpers are bools, triggers are doubles)
    private double p1LeftBumperToDouble;
    private double p1RightBumperToDouble;
    private double p2LeftBumperToDouble;
    private double p2RightBumperToDouble;
    private final double triggerSense = 0.4;
    
    // Default values, wantedPattern is the pattern for the next 3 balls, indexerPattern is what is in the indexer
    // possible values for indexerPattern=empty, purple, green, unknown(there is a ball, we don't know what color)
    private String[] wantedPattern = {"purple", "green", "purple"}
    private String[] indexerPattern = {"empty", "empty", "empty"}
    private boolean patternChecking = false;
    
    // Software indexing state (disabled: color sensor-based indexing)
    /*
    private enum BallColor { BLUE, PURPLE, UNKNOWN }
    private BallColor[] slots = new BallColor[] { BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN };
    private int head = 0; // slot at shooting/color sensor position
    private int pendingSteps = 0; // queued forward steps
    private boolean wasMovingLastUpdate = false;
    private NormalizedColorSensor colorSensor;
    */


    @Override
    public void init() 
    {
        HardwareMap hw = hardwareMap;
        // Use Pedro Pathing Follower for teleop drive
        drive = new PedroDrive(hw);
        turret = new TurretSubsystem(hw, TURRET, TURRET_ANGLE);
        intake = new IntakeSubsystem(hw, INTAKE, INTAKE_ANGLE, RAMP);
        indexer = new IndexerSubsystem(hw, INDEXER, FEED_LEVER);
        flywheel = new FlywheelSubsystem(hw, FLYWHEEL);
        // Enable dashboard configurables for indexer presets
        try { PanelsConfigurables.INSTANCE.refreshClass(indexer); } catch (Exception ignore) {}
        // Color sensor disabled
        // try {
        //     colorSensor = hw.get(NormalizedColorSensor.class, COLOR_SENSOR);
        // } catch (Exception ignore) { colorSensor = null; }
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


        // toggle override after 1 second
        if (gamepad2.right_trigger > triggerSense)
        {
            if (overrideTime.seconds() >= 1)
            {
                override = !override;
                overrideTime.reset();
            }
        }
        else
        {
            overrideTime.reset()
        }

        
        // Read drive inputs (FTC sticks: up is -y)
        double y = -gamepad1.left_stick_y;  // forward
        double x = gamepad1.left_stick_x;   // strafe
        double rx = gamepad1.right_stick_x; // rotation

        drive.setDriverInput(x, y, rx, false);
        drive.update();

        hoodInput = gamepad2.left_stick_y;
        turretInput = gamepad2.right_stick_x;
        
        // Mechanisms
        // Turret: rotate with right_stick_x, angle with left_stick_y
        // Zach working on it, make sure to move to override/non-override code
        turret.setManualInput(gamepad2.right_stick_x);
        turret.setAngleInput(gamepad2.left_stick_y);
        turret.update();

        // intaking
        intake.setTriggers(p1LeftBumperToDouble, gamepad1.left_trigger);
        if (!intakeHold)
        {
            if (gamepad1.right_trigger > triggerSens)
            {
                intake.setRotationInput(-1);
            }
            else if (gamepad1.right_bumper)
            {
                intake.setRotationInput(1);
            }
        }


        // indexer to intake positions
        if (gamepad1.dpad_left && prevIndex != "P1Left")
        {
            indexer.setIndexerPosition("Intake1");
            intakeHold = true;
            intake.setRotationInput(-1);
        }
        else if (gamepad1.dpad_down && prevIndex != "P1Down")
        {
            indexer.setIndexerPosition("Intake2"); 
            intakeHold = true;
            intake.setRotationInput(-1);
        }
        else if (gamepad1.dpad_right && prevIndex != "P1Right")
        {
            indexer.setIndexerPosition("Intake3");
            intakeHold = true;
            intake.setRotationInput(-1);
        }

        // Override button
        if (override)
        {
            // indexer to launch positions
            if (gamepad2.dpad_left && prevIndex != "P2Left")
            {
                indexer.setIndexerPosition("Launch1");
                intakeHold = true;
                intake.setRotationInput(-1);
            }
            else if (gamepad2.dpad_down && prevIndex != "P2Down")
            {
                indexer.setIndexerPosition("Launch2");
                intakeHold = true;
                intake.setRotationInput(-1);
            }
            else if (gamepad2.dpad_right && prevIndex != "P2Right")
            {
                indexer.setIndexerPosition("Launch3");
                intakeHold = true;
                intake.setRotationInput(-1);
            }
            
            // ramp control
            if (gamepad2.left_bumper)
            {
                // lower ramp
                intake.setRampPosition(-1);
            }
            else
            {
                // raise ramp
                intake.setRampPosition(1);
            }

             // TODO: manual turret, hood, motor close/far(no hood)
        }
        else
        {
            // auto ramp control
            if ((gamepad1.left_bumper || gamepad1.left_trigger > triggerSense) && !intakeHold)
            {
                // lower ramp
                intake.setRampPosition(-1);
            }
            else
            {
                // raise ramp
                intake.setRampPosition(1);
            }

            // launch sequence
            if (gamepad2.y && !patternChecking)
            {
                // launching sequence in subsystem
            }

            // set wanted pattern
            if (gamepad2.x)
            {
                wantedPattern = {"green", "purple", "purple"}
            }
            else if (gamepad2.a)
            {
                wantedPattern = {"purple", "green", "purple"}
            }
            else if (gamepad2.b)
            {
                wantedPattern = {"purple", "purple", "green"}
            }

            if (gamepad2.x || gamepad2.a || gamepad2.b)
            {
                // check pattern in indexer
            }

            // TODO: motor close/far with hood auto aim
        }

        // Maintain lever timing and magnet
        // pulse launch arm
        indexer.handleLeverButton(gamepad2.dpad_up);
        indexer.update();

            // Normal auto-release of intake hold when indexer finishes
            if (!indexer.isMoving()) 
            {
                intakeHold = false;
            }

        // Telemetry
        telemetry.addData("Override State", override);
        telemetry.addData("magnetState", indexer.getMagnetState());
        telemetry.addData("EncoderTicks", indexer.getCurrentPosition());
        telemetry.addData("Drive", "x=%.2f y=%.2f rx=%.2f", x, y, rx);
        telemetry.addData("Turret", turret.getStatus());
        telemetry.addData("Intake", intake.getStatus());
        telemetry.addData("Indexer", indexer.getStatus());
        telemetry.addData("Indexer Enc", String.format("cur=%d tgt=%d",
            indexer.getCurrentPosition(),
            indexer.getTargetPosition()));
        // telemetry.addData("Buffer", String.format("head=%d slots=[%s,%s,%s]", head, slots[0], slots[1], slots[2]));
        telemetry.addData("Flywheel", flywheel.getStatus());
        telemetry.update();


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
    }
}
// Old code(seems to be for color detection mainly), might use later


        // boolean moving = indexer.isMoving();
        // if (wasMovingLastUpdate && !moving) {
        //     // Just arrived: sample color at head
        //     sampleAndStoreColorAtHead();
        // }
        // wasMovingLastUpdate = moving;
        // if (!moving && pendingSteps > 0) {
        //     // Execute a single forward step through POSITION_1 -> POSITION_2 -> POSITION_3 -> POSITION_1
        //     IndexerSubsystem.Selection sel = indexer.getSelection();
        //     IndexerSubsystem.Selection nextSel;
        //     switch (sel) {
        //         case POSITION_1:
        //             nextSel = IndexerSubsystem.Selection.POSITION_2; break;
        //         case POSITION_2:
        //             nextSel = IndexerSubsystem.Selection.POSITION_3; break;
        //         case POSITION_3:
        //         default:
        //             nextSel = IndexerSubsystem.Selection.POSITION_1; break;
        //     }
        //     indexer.setSelection(nextSel);
        //     head = (head + 1) % 3;
        //     pendingSteps--;
        // }


// private int findColorIndex(BallColor desired) {
    //     for (int i = 0; i < 3; i++) {
    //         if (slots[i] == desired) return i;
    //     }
    //     return -1;
    // }


// private void sampleAndStoreColorAtHead() {
    //     if (colorSensor == null) return;
    //     try {
    //         NormalizedRGBA colors = colorSensor.getNormalizedColors();
    //         final float[] hsv = new float[3];
    //         Color.colorToHSV(colors.toColor(), hsv);
    //         float hue = hsv[0];
    //         BallColor detected;
    //         if (hue >= 200 && hue < 260) {
    //             detected = BallColor.BLUE;
    //         } else if (hue >= 260 && hue < 340) {
    //             detected = BallColor.PURPLE;
    //         } else {
    //             detected = BallColor.UNKNOWN;
    //         }
    //         slots[head] = detected;
    //     } catch (Exception ignore) { /* leave UNKNOWN */ }
    // }


// Color selection disabled
        // if (gamepad1.x && !prevX) {
        //     intake.setHoldUp(true);
        //     int targetIndex = findColorIndex(BallColor.BLUE);
        //     if (targetIndex >= 0) {
        //         int deltaForward = (targetIndex - head + 3) % 3;
        //         pendingSteps += deltaForward;
        //     }
        // } else if (gamepad1.b && !prevB) {
        //     intake.setHoldUp(true);
        //     int targetIndex = findColorIndex(BallColor.PURPLE);
        //     if (targetIndex >= 0) {
        //         int deltaForward = (targetIndex - head + 3) % 3;
        //         pendingSteps += deltaForward;
        //     }
        // }
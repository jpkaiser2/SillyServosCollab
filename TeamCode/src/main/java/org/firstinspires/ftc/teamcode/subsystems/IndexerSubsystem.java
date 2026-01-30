package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class IndexerSubsystem {
    public enum Selection { POSITION_1, POSITION_2, POSITION_3 }

    @IgnoreConfigurable
    private final DcMotorEx indexerMotor;    // motor with encoder controlling indexer
    @IgnoreConfigurable
    private final Servo feedLeverServo;      // lever that feeds balls into intake
    @IgnoreConfigurable
    private final TouchSensor magnetSensor;
    // Manual mode removed

    // Magnet sensor logic

    // switch to true if we start the robot with the magnet, needs to be accurate
    private static boolean previousMagnetState = false;
    // current encoder value needs to be +- this amount to expected to regester the magnet
    public static final int approxEncoderAccuracy;
    public static int magnetBasedOffset;

    // default value in case we start on the magnet, may be slightly inaccurate, but should fix itself after 1 rotation
    public static int magnetRisingEdgePosition = 0;

    // magnet positions in encoder ticks(note: define this position as the encoder tick when the magnet is centered)
    // !!Warning!! these are guessed values, need to update before using
    public static int magnetPosition1 = 0;
    public static int magnetPosition2 = 192;

    // Preset positions in encoder ticks (user-provided; tune as needed)
    public static int POSITION_1 = 0;
    public static int POSITION_2 = 192;
    public static int POSITION_3 = 94;

    // Secondary collection positions in encoder ticks
    public static int COLLECTION_1 = 429;
    public static int COLLECTION_2 = 332;
    public static int COLLECTION_3 = 241;

    @IgnoreConfigurable
    private Selection selection = Selection.POSITION_2; // default to middle

    // Lever pulse config
    @IgnoreConfigurable
    private final ElapsedTime leverTimer = new ElapsedTime();
    @IgnoreConfigurable
    private boolean leverPulsing = false;
    @IgnoreConfigurable
    private long leverPulseMs = 200;
    @IgnoreConfigurable
    private double leverIdlePos = 0.2;
    @IgnoreConfigurable
    private double leverEngagedPos = 0.8;
    @IgnoreConfigurable
    private double leverMaxPos = 0.6; // cap the physical max position

    public IndexerSubsystem(HardwareMap hardwareMap, String indexerMotorName, String feedLeverServoName) {
        this.indexerMotor = hardwareMap.get(DcMotorEx.class, indexerMotorName);
        this.indexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.indexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.indexerMotor.setPower(0.0);

        this.feedLeverServo = hardwareMap.get(Servo.class, feedLeverServoName);
        this.feedLeverServo.setDirection(Servo.Direction.REVERSE);
        this.feedLeverServo.setPosition(Math.min(leverIdlePos, leverMaxPos));
    }

    public void updateMagnet()
    {
        // rising edge
        if (!previousMagnetState && magnetSensor.isPressed())
        {
            // at magnet 1 or 2
            if((indexerMotor.getCurrentPosition() > magnetPosition1 - approxEncoderAccuracy) && (indexerMotor.getCurrentPosition() < magnetPosition1 + approxEncoderAccuracy)
              || (indexerMotor.getCurrentPosition() > magnetPosition2 - approxEncoderAccuracy) && (indexerMotor.getCurrentPosition() < magnetPosition2 + approxEncoderAccuracy))
            {
                magnetRisingEdgePosition = indexerMotor.getCurrentPosition();
            }
        }

        // falling edge
        else if (previousMagnetState && !magnetSensor.isPressed())
        {
            // at magnet 1
            if((indexerMotor.getCurrentPosition() > magnetPosition1 - approxEncoderAccuracy) && (indexerMotor.getCurrentPosition() < magnetPosition1 + approxEncoderAccuracy))
            {
                // calculate average to find encoder position of the magnet, not the rising/falling edge of it
                int average = (int) ((magnetRisingEdgePosition + indexerMotor.getCurrentPosition()) / 2.0);
                magnetBasedOffset = magnetPosition1 - average;
                POSITION_1 += magnetBasedOffset;
                POSITION_2 += magnetBasedOffset;
                POSITION_3 += magnetBasedOffset;
                COLLECTION_1 += magnetBasedOffset;
                COLLECTION_2 += magnetBasedOffset;
                COLLECTION_3 += magnetBasedOffset;
                magnetPosition1 += magnetBasedOffset;
                magnetPosition2 += magnetBasedOffset;
            }

            // at magnet 2
            else if((indexerMotor.getCurrentPosition() > magnetPosition2 - approxEncoderAccuracy) && (indexerMotor.getCurrentPosition() < magnetPosition2 + approxEncoderAccuracy))
            {
                // calculate average to find encoder position of the magnet, not the rising/falling edge of it
                int average = (int) ((magnetRisingEdgePosition + indexerMotor.getCurrentPosition()) / 2.0);
                magnetBasedOffset = magnetPosition2 - average;
                POSITION_1 += magnetBasedOffset;
                POSITION_2 += magnetBasedOffset;
                POSITION_3 += magnetBasedOffset;
                COLLECTION_1 += magnetBasedOffset;
                COLLECTION_2 += magnetBasedOffset;
                COLLECTION_3 += magnetBasedOffset;
                magnetPosition1 += magnetBasedOffset;
                magnetPosition2 += magnetBasedOffset;
            }
        }
    // update previous state for falling/rising edge
    previousMagnetState = magnetSensor.isPressed();
    }

    public void setLeverConfig(long pulseMs, double idle, double engaged) {
        leverPulseMs = pulseMs;
        leverIdlePos = Math.min(idle, leverMaxPos);
        leverEngagedPos = Math.min(engaged, leverMaxPos);
    }

    /** Set a hard maximum position for the feed lever servo (0..1). */
    public void setLeverMax(double max) {
        leverMaxPos = Math.max(0.0, Math.min(1.0, max));
        leverIdlePos = Math.min(leverIdlePos, leverMaxPos);
        leverEngagedPos = Math.min(leverEngagedPos, leverMaxPos);
        if (!leverPulsing) {
            feedLeverServo.setPosition(leverIdlePos);
        }
    }

    /** Choose which preset aligns with the turret. */
    public void setSelection(Selection sel) {
        this.selection = sel;
        int target;
        switch (sel) {
            case POSITION_1:
                target = POSITION_1; break;
            case POSITION_2:
                target = POSITION_2; break;
            case POSITION_3:
            default:
                target = POSITION_3; break;
        }
        indexerMotor.setTargetPosition(target);
        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexerMotor.setPower(0.6); // move power; tune as needed
    }

    /** Choose which collection preset to move to. */
    public void setCollectionSelection(Selection sel) {
        int target;
        switch (sel) {
            case POSITION_1:
                target = COLLECTION_1; break;
            case POSITION_2:
                target = COLLECTION_2; break;
            case POSITION_3:
            default:
                target = COLLECTION_3; break;
        }
        indexerMotor.setTargetPosition(target);
        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexerMotor.setPower(0.6);
    }

    public Selection getSelection() { return selection; }

    // Manual mode APIs removed

    /** Call once per loop to maintain lever pulse timing. */
    public void update() {
        updateLever();
        updateMagnet();
    }

    /** Trigger feed lever pulse on button press (e.g., gamepad2.y). */
    public void handleLeverButton(boolean pressed) {
        if (pressed && !leverPulsing) {
            leverPulsing = true;
            leverTimer.reset();
            feedLeverServo.setPosition(Math.min(leverEngagedPos, leverMaxPos));
        }
    }

    /** Non-blocking lever update, return to idle after pulse. */
    public void updateLever() {
        if (leverPulsing && leverTimer.milliseconds() >= leverPulseMs) {
            feedLeverServo.setPosition(Math.min(leverIdlePos, leverMaxPos));
            leverPulsing = false;
        }
    }

    public String getStatus() {
        boolean busy = indexerMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && indexerMotor.isBusy();
        return String.format("indexerSel=%s mode=%s busy=%s leverPulsing=%s",
            selection, indexerMotor.getMode(), busy, leverPulsing);
    }

    /** Whether the indexer is currently moving toward a target position. */
    public boolean isMoving() {
        return indexerMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && indexerMotor.isBusy();
    }

    /** Current encoder position of the indexer motor. */
    public int getCurrentPosition() {
        return indexerMotor.getCurrentPosition();
    }

    /** Target encoder position when running to position. */
    public int getTargetPosition() {
        return indexerMotor.getTargetPosition();
    }

    /** Nudge the indexer target by a number of encoder ticks. */
    public void nudgeTicks(int deltaTicks) {
        int base;
        if (indexerMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            base = indexerMotor.getTargetPosition();
        } else {
            base = indexerMotor.getCurrentPosition();
        }
        int target = base + deltaTicks;
        indexerMotor.setTargetPosition(target);
        try { indexerMotor.setTargetPositionTolerance(1); } catch (Exception ignore) {}
        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexerMotor.setPower(0.3);
    }

    // Tuning helpers removed
}
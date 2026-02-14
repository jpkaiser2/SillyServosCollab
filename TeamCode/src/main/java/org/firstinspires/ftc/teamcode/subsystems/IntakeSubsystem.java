package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeSubsystem {
    private final DcMotorEx intakeMotor; // Core Hex motor
    private final Servo intakeAngleServo; // rotates intake
    private Servo rampServo; // ramp servo

    // Holds the last commanded intake angle position (0..1)
    private double intakeAnglePos = 0.5;
    private static final double ANGLE_DEADBAND = 0.05; // stick deadband to hold position
    private static final double INTAKE_UP_POS = 0.75; // up position to avoid interference
   
    // Ramp direction: min = up, max = down
    private static final double RAMP_UP_POS = 0.2; // slightly above absolute min to avoid overtravel
    private static final double RAMP_DOWN_POS = 1.0;  // full down

    // When true, intakeAngleServo is forced to the up position and held
    private boolean holdUp = false;
    private boolean requestStageFlag = false;
    private boolean prevStageButton = false;

    public IntakeSubsystem(HardwareMap hardwareMap, String intakeMotorName, String intakeAngleServoName) {
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.intakeAngleServo = hardwareMap.get(Servo.class, intakeAngleServoName);
        this.intakeAngleServo.setPosition(intakeAnglePos);
        // Ramp servo not provided via this constructor
        this.rampServo = null;
    }

    //Overloaded constructor to include ramp servo control.
    public IntakeSubsystem(HardwareMap hardwareMap,
                           String intakeMotorName,
                           String intakeAngleServoName,
                           String rampServoName) {
        this(hardwareMap, intakeMotorName, intakeAngleServoName);
        try {
            this.rampServo = hardwareMap.get(Servo.class, rampServoName);
            // Initialize ramp to MIN (up) per updated direction
            this.rampServo.setPosition(RAMP_UP_POS);
        } catch (Exception ignore) {
            // If not found, leave rampServo as null
            this.rampServo = null;
        }
    }

    /**
     * Update intake motor based on trigger inputs.
     * @param inTrigger right trigger (intake forward)
     * @param outTrigger left trigger (reverse)
     */
    public void setTriggers(double inTrigger, double outTrigger) {
        double power = 0.0;
        if (inTrigger > 0.05 && outTrigger < 0.05) {
            power = Math.min(1.0, inTrigger);
        } else if (outTrigger > 0.05 && inTrigger < 0.05) {
            power = -Math.min(1.0, outTrigger);
        } else {
            power = 0.0;
        }
        intakeMotor.setPower(power);
    }

    /**
     * Adjust intake rotation servo (e.g., gamepad2.left_stick_x).
     * When the stick is inside the deadband, the servo holds its last position.
     */
    public void setRotationInput(double joystick) {
        if (holdUp) {
            // While held up, keep servo at the up position
            intakeAnglePos = INTAKE_UP_POS;
            intakeAngleServo.setPosition(intakeAnglePos);
            return;
        }
        double j = Math.max(-1.0, Math.min(1.0, joystick));
        if (Math.abs(j) > ANGLE_DEADBAND) {
            double pos = (j * 0.5) + 0.5; // -1..1 -> 0..1
            intakeAnglePos = Math.max(0.0, Math.min(1.0, pos));
            intakeAngleServo.setPosition(intakeAnglePos);
        }
        // else: hold previous intakeAnglePos
    }


    /** Edge-detect stage request button (e.g., gamepad2.a) */
    public void handleStageButton(boolean pressed) {
        if (pressed && !prevStageButton) {
            requestStageFlag = true;
        }
        prevStageButton = pressed;
    }

    /** Whether a stage request is active. */
    public boolean requestStage() {
        return requestStageFlag;
    }

    /** Clear the stage request flag once consumed. */
    public void clearRequestStage() {
        requestStageFlag = false;
    }

    public String getStatus() {
        double rampPos = (rampServo != null) ? rampServo.getPosition() : -1.0;
        return String.format(
                "intakePower=%.2f, intakeAngle=%.2f, ramp=%.2f, requestStage=%s",
                intakeMotor.getPower(), intakeAnglePos, rampPos, requestStageFlag);
    }

    /** Enable or disable holding the intake angle in the up position. */
    public void setHoldUp(boolean enable) {
        holdUp = enable;
        if (holdUp) {
            intakeAnglePos = INTAKE_UP_POS;
            intakeAngleServo.setPosition(intakeAnglePos);
        }
    }

    /** Whether the intake angle is currently being held up. */
    public boolean isHoldUp() { return holdUp; }

    // Ramp controls
    /** Set ramp to minimum (up). */
    public void setRampUp() {
        if (rampServo != null) {
            rampServo.setPosition(RAMP_UP_POS);
        }
    }

    /** Set ramp to maximum (down). */
    public void setRampDown() {
        if (rampServo != null) {
            rampServo.setPosition(RAMP_DOWN_POS);
        }
    }

    /** Get current ramp position, or -1 if unavailable. */
    public double getRampPosition() {
        return (rampServo != null) ? rampServo.getPosition() : -1.0;
    }
}

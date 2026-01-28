package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Raw mecanum drive implementation using simple motor mixing.
 * TODO: Adjust motor directions if they don't match your build.
 */
public class RawMecanumDrive implements DriveBase {
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final String imuName; // retained for signature compatibility, unused

    // Driver inputs cached between calls
    private double x;     // strafe
    private double y;     // forward
    private double rx;    // rotation
    private boolean fieldCentric;

    public RawMecanumDrive(HardwareMap hardwareMap,
                           String frontLeftName,
                           String frontRightName,
                           String backLeftName,
                           String backRightName,
                           String imuName) {
        this.frontLeft = hardwareMap.get(DcMotorEx.class, frontLeftName);
        this.frontRight = hardwareMap.get(DcMotorEx.class, frontRightName);
        this.backLeft = hardwareMap.get(DcMotorEx.class, backLeftName);
        this.backRight = hardwareMap.get(DcMotorEx.class, backRightName);
        this.imuName = imuName;

        // Typical mecanum: reverse left side motors.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Run without encoders for simple teleop control.
        frontLeft.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Robot-centric only: no IMU initialization.
    }

    @Override
    public void setDriverInput(double x, double y, double rx, boolean fieldCentric) {
        this.x = x;
        this.y = y;
        this.rx = rx;
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void update() {
        double ix = x;
        double iy = y;

        double fl = iy + ix + rx;
        double fr = iy - ix - rx;
        double bl = iy - ix + rx;
        double br = iy + ix - rx;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}

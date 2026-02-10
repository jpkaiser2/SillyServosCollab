package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.AngleUnit;

public class FlywheelSubsystem {
    private final DcMotorEx flywheel;
    public FlywheelSubsystem(HardwareMap hardwareMap, String flywheelName) {
        this.flywheel = hardwareMap.get(DcMotorEx.class, flywheelName);
        // Flywheel typically allowed to coast
        this.flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        power = Math.max(-1.0, Math.min(1.0, power));
        flywheel.setPower(power);
    }
    public double getPower() {
        return flywheel.getPower();
    }
    public double getVelocity() {
        return flywheel.getVelocity();
    }
    public void setVelocity(double angularVelocity) {
        flywheel.setVelocity(angularVelocity, AngleUnit.DEGREES);
    }

    public String getStatus() {
        return String.format("flywheelPower=%.2f flywheelVelocity=%.2f", flywheel.getPower(), flywheel.getVelocity());
    }
}

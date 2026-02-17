package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FlywheelSubsystem {
    private final DcMotorEx flywheel;
    private static final double DEFAULT_RPS = 1.6; // target constant speed

    public FlywheelSubsystem(HardwareMap hardwareMap, String flywheelName) {
        this.flywheel = hardwareMap.get(DcMotorEx.class, flywheelName);
        // Flywheel typically allowed to coast
        this.flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Use built-in velocity control via encoder for setVelocity()
        this.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        // I think this is ticks/second
        return flywheel.getVelocity();
    }

    public double getVelocityRps() {
        // convert to rotations per second for easier comparison
        return flywheel.getVelocity(AngleUnit.DEGREES) / 360.0;
    }

    public void setVelocity(double angularVelocity) {
        // in rot/second
        flywheel.setVelocity(angularVelocity * 360, AngleUnit.DEGREES);
    }

    public void holdConstantSpeed() {
        setVelocity(DEFAULT_RPS);
    }

    public String getStatus() {
        return String.format("flywheelPower=%.2f flywheelRPS=%.2f", flywheel.getPower(), getVelocityRps());
    }
}

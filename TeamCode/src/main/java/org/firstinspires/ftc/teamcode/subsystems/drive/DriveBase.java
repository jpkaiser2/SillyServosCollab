package org.firstinspires.ftc.teamcode.subsystems.drive;

public interface DriveBase {
    /**
     * Set driver inputs.
     * @param x strafe input (-1..1)
     * @param y forward input (-1..1)
     * @param rx rotation input (-1..1)
     * @param fieldCentric true for field-centric driving when IMU available
     */
    void setDriverInput(double x, double y, double rx, boolean fieldCentric);

    /**
     * Apply the current inputs to the drive hardware. Call once per loop.
     */
    void update();
}

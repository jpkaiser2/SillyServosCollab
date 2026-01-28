package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * PedroDrive wraps the Pedro Pathing Follower for TeleOp control.
 * Implements the DriveBase interface so it can drop-in replace RawMecanumDrive.
 */
public class PedroDrive implements DriveBase {
    private final Follower follower;

    // Cached driver inputs between update calls
    private double x; // strafe
    private double y; // forward
    private double rx; // rotation
    private boolean fieldCentric;
    private Boolean lastFieldCentric = null;

    public PedroDrive(HardwareMap hardwareMap) {
        this.follower = Constants.createFollower(hardwareMap);
        // Enable TeleOp drive mode in field-centric configuration.
        try {
            follower.startTeleopDrive(true);
        } catch (Throwable t) {
            // Fallback in case only the no-arg overload exists
            try { follower.startTeleopDrive(); } catch (Throwable ignored) {}
        }
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
        // Ensure follower mode matches requested centricity
        if (lastFieldCentric == null || lastFieldCentric != fieldCentric) {
            try { follower.startTeleopDrive(fieldCentric); } catch (Throwable ignored) {}
            lastFieldCentric = fieldCentric;
        }
        // Unified axis mapping for both modes:
        // forward = y, strafe = -x, turn = -rx
        follower.setTeleOpDrive(y, -x, -rx, fieldCentric);
        follower.update();
    }
}

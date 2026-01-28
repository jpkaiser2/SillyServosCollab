package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "AprilTag Webcam Example RED")
public class AprilTagWebcamExample extends OpMode {

    // Subsystems
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private TurretSubsystem turret;

    // Hardware names
    private static final String TURRET_MOTOR = "turret";
    private static final String TURRET_ANGLE_SERVO = "turretAngle";

    // Locked target tag ID
    private static final int TARGET_TAG_ID = 24;

    // ===== PD/PID tuning (bearing in degrees, motor power output) =====
    private static final double kP = 0.020;     // power per degree
    private static final double kI = 0.000;     // usually 0 for turrets
    private static final double kD = 0.0025;    // power per (deg/sec) using turret encoder velocity

    private static final double DEADBAND_DEG = 1.0;
    private static final double MAX_POWER = 0.60;

    // Helps overcome stiction so small errors still move
    private static final double kS = 0.06;

    // Bearing low-pass filter: higher = smoother/slower
    private static final double BEARING_FILTER_ALPHA = 0.70;

    // Output slew rate limit: power per second
    private static final double MAX_POWER_SLEW_PER_SEC = 2.0;

    // Flip if turret rotates the wrong direction
    private static final double MOTOR_SIGN = 1.0;

    // Tag dropout handling
    private static final double TAG_TIMEOUT_SEC = 0.25;

    // If ftcPose is missing, use pixel fallback (approx HFOV, tune if you know your camera)
    private static final int IMAGE_WIDTH_PX = 640;
    private static final double CAMERA_HFOV_DEG = 70.0; // rough for many webcams; replace with your webcam spec if known

    // ===== State =====
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime tagTimer = new ElapsedTime();

    private double bearingFiltDeg = 0.0;
    private double lastTurretAngleDeg = 0.0;
    private double integral = 0.0;

    private double lastCmdPower = 0.0;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        aprilTagWebcam.setTargetTagId(TARGET_TAG_ID);

        turret = new TurretSubsystem(hardwareMap, TURRET_MOTOR, TURRET_ANGLE_SERVO);
        turret.setMaxPower(MAX_POWER);

        // Soft-zero turret encoder at init position.
        // Best practice: physically point turret forward during init, then press INIT.
        turret.zeroTurretHere();

        loopTimer.reset();
        tagTimer.reset();
        lastTurretAngleDeg = turret.getTurretAngleDeg();

        telemetry.addLine("Locked to AprilTag ID 21. Camera is on turret -> aiming drives bearing to 0.");
        telemetry.addLine("If turret turns wrong way, flip MOTOR_SIGN.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0.0) dt = 0.02;

        aprilTagWebcam.update();

        // Only track the locked ID (21). No fallback selection.
        AprilTagDetection tag = aprilTagWebcam.getTargetTag();

        double cmdPower = 0.0;

        if (tag != null) {
            tagTimer.reset();

            // Bearing measurement (deg)
            double bearingDeg;
            if (tag.ftcPose != null) {
                bearingDeg = tag.ftcPose.bearing;
            } else {
                // Pixel fallback only if needed
                bearingDeg = AprilTagWebcam.estimateBearingFromPixels(
                        tag.center.x, IMAGE_WIDTH_PX, CAMERA_HFOV_DEG
                );
            }

            // Low-pass filter bearing to reduce jitter
            bearingFiltDeg = BEARING_FILTER_ALPHA * bearingFiltDeg
                    + (1.0 - BEARING_FILTER_ALPHA) * bearingDeg;

            // Error = desired(0) - measured(bearing)
            double errorDeg = -bearingFiltDeg;

            boolean inDeadband = Math.abs(errorDeg) < DEADBAND_DEG;
            if (inDeadband) errorDeg = 0.0;

            // Derivative on measurement (turret angular velocity from encoder)
            double turretAngleDeg = turret.getTurretAngleDeg();
            double turretVelDegPerSec = (turretAngleDeg - lastTurretAngleDeg) / dt;
            lastTurretAngleDeg = turretAngleDeg;

            // Integral (optional)
            if (kI != 0.0 && !inDeadband) {
                integral += errorDeg * dt;
                integral = clip(integral, -50.0, 50.0);
            } else if (inDeadband) {
                integral *= 0.8;
            }

            // PD/PID output
            double u = (kP * errorDeg) + (kI * integral) - (kD * turretVelDegPerSec);

            // Static friction compensation
            if (Math.abs(u) > 1e-4) {
                u += Math.signum(u) * kS;
            }

            // Clamp
            u = clip(u, -MAX_POWER, MAX_POWER);

            // Slew-rate limit
            u = slewLimit(u, lastCmdPower, MAX_POWER_SLEW_PER_SEC, dt);
            lastCmdPower = u;

            cmdPower = MOTOR_SIGN * u;

        } else {
            // Tag not visible: stop after timeout
            if (tagTimer.seconds() > TAG_TIMEOUT_SEC) {
                cmdPower = 0.0;
                integral *= 0.9;
            }
        }

        turret.setTurretPower(cmdPower);
        turret.update();

        // Telemetry
        telemetry.addData("TargetTagID", TARGET_TAG_ID);
        telemetry.addData("TagVisible", tag != null);
        telemetry.addData("BearingFilt(deg)", bearingFiltDeg);
        telemetry.addData("TurretAngle(deg)", turret.getTurretAngleDeg());
        telemetry.addData("CmdPower", cmdPower);
        telemetry.addData("TagAge(s)", tagTimer.seconds());
        if (tag != null) aprilTagWebcam.displayDetectionTelemetry(tag);
        telemetry.addData("TurretStatus", turret.getStatus());
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        aprilTagWebcam.stop();
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double slewLimit(double target, double current, double maxDeltaPerSec, double dt) {
        double maxDelta = maxDeltaPerSec * dt;
        double delta = clip(target - current, -maxDelta, maxDelta);
        return current + delta;
    }
}

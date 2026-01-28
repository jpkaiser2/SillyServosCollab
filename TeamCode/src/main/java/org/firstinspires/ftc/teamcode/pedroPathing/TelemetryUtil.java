package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class TelemetryUtil {
    public static void emitPose(TelemetryManager panelsTelemetry, Follower follower) {
        if (panelsTelemetry == null || follower == null) return;
        Pose pose = follower.getPose();
        panelsTelemetry.debug("Pose/X", pose.getX());
        panelsTelemetry.debug("Pose/Y", pose.getY());
        panelsTelemetry.debug("Pose/HeadingDeg", Math.toDegrees(pose.getHeading()));
    }
}

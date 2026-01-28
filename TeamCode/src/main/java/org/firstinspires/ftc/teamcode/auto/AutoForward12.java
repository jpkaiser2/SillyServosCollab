package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto Forward 12in (Pedro)", group = "Auto")
public class AutoForward12 extends OpMode {
    private Follower follower;
    private Path forward12;
    private boolean started = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // Start at origin, heading 0 deg; units are inches per Constants
        follower.setStartingPose(new Pose(0, 0));
    }

    @Override
    public void start() {
        // Build a simple straight line 12 inches forward along +X with constant heading 0
        forward12 = new Path(new BezierLine(new Pose(0, 0), new Pose(12, 0)));
        forward12.setConstantHeadingInterpolation(0);
        follower.activateAllPIDFs();
        follower.followPath(forward12);
        started = true;
    }

    @Override
    public void loop() {
        follower.update();

        // Telemetry for confidence
        telemetry.addData("Pose", String.format("x=%.2f y=%.2f h=%.2f",
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading()));
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();

        // Stop driving once finished
        if (started && !follower.isBusy()) {
            follower.startTeleopDrive(true);
            follower.setTeleOpDrive(0, 0, 0, true);
            started = false;
        }
    }
}

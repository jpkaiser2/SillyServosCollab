package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoPathStateMachine;
import org.firstinspires.ftc.teamcode.pedroPathing.TelemetryUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsFieldUtil;

@Autonomous(name = "pedroTestShoot", group = "Autonomous")
@Configurable // Panels
public class pedroTestShoot extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private AutoPathStateMachine auto; // Reusable path state machine
    private boolean autoStopped; // Whether we've switched to teleop stop mode

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        auto = new AutoPathStateMachine(follower)
            .add(paths.Path1)
            .add(paths.Path2)
            .add(paths.Path3);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        // Prepare Panels Field drawing coordinates for Pedro
        PanelsFieldUtil.initPedroOffsets();
        autoStopped = false;
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        if (auto.getState() == AutoPathStateMachine.State.IDLE) {
            auto.start();
        }
        auto.update();
        pathState = auto.getIndex();

        // When all paths are complete, stop the robot
        if (auto.isComplete() && !follower.isBusy()) {
            if (!autoStopped) {
                follower.startTeleopDrive(true);
                panelsTelemetry.debug("Status", "Auto Complete - Stopping");
                autoStopped = true;
            }
        }

        // Hold zero power while stopped
        if (autoStopped) {
            follower.setTeleOpDrive(0, 0, 0, true);
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        TelemetryUtil.emitPose(panelsTelemetry, follower);
        panelsTelemetry.update(telemetry);

        // Draw planned paths and current robot pose on Panels Field
        for (PathChain chain : auto.getPaths()) {
            PanelsFieldUtil.drawPathChain(chain);
        }
        PanelsFieldUtil.drawRobot(follower);
        PanelsFieldUtil.update();
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.595, 16.000), new Pose(53.931, 44.293))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.931, 44.293), new Pose(53.931, 103.413))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.931, 103.413), new Pose(38.178, 116.201))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() { return pathState; }
}

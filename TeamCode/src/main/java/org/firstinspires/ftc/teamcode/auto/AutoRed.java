package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

@Autonomous(name = "Auto Red", group = "Auto")
public class AutoRed extends OpMode {
    private Follower follower;
    private IndexerSubsystem indexer;
    private LaunchSubsystem launch;
    private FlywheelSubsystem flywheel;
    
    private Path reverseThreeFeet;
    private Path lateralMove;
    private boolean pathStarted = false;
    private boolean pathComplete = false;
    private boolean launchInitiated = false;
    private boolean launchComplete = false;
    private boolean lateralMoveStarted = false;
    private boolean lateralMoveComplete = false;
    private long launchStartTime = 0;
    
    // Lever pulse tracking
    private int leverPulseCount = 0;
    private boolean firstPulseDone = false;
    private long firstPulseCompleteTime = 0;
    private boolean flywheelStopped = false;
    private long flywheelSpinUpStartTime = 0;

    // Timeout for launch sequence (5 seconds)
    private static final long LAUNCH_TIMEOUT_MS = 5000;
    private static final long PULSE_DELAY_MS = 250; // Delay between pulses
    private static final long FLYWHEEL_SPIN_UP_TIME_MS = 3000; // 3 seconds for flywheel to reach speed
    private static final long POST_LAUNCH_SPIN_TIME_MS = 1000; // 1 second after launch completes

    @Override
    public void init() {
        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0));

        // Initialize subsystems
        indexer = new IndexerSubsystem(hardwareMap, "indexer", "feedLever");
        flywheel = new FlywheelSubsystem(hardwareMap, "flywheel");
        launch = new LaunchSubsystem(indexer);
    }

    @Override
    public void start() {
        // Build path: reverse 3 feet
        // Moving along negative X-axis (backwards)
        reverseThreeFeet = new Path(new BezierLine(new Pose(0, 0), new Pose(-50, 0)));
        reverseThreeFeet.setConstantHeadingInterpolation(0);
        
        follower.activateAllPIDFs();
        follower.followPath(reverseThreeFeet);
        pathStarted = true;

        // Reset launch state for a clean run
        pathComplete = false;
        launchInitiated = false;
        launchComplete = false;
        lateralMoveStarted = false;
        lateralMoveComplete = false;
        launchStartTime = 0;
        leverPulseCount = 0;
        firstPulseDone = false;
        firstPulseCompleteTime = 0;
        flywheelStopped = false;
        flywheelSpinUpStartTime = 0;
        
        // Spin up flywheel
        flywheel.holdConstantSpeed();
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("Status", "Running");
        telemetry.addData("Pose", String.format("x=%.2f y=%.2f h=%.2f",
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading()));
        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.addData("Launching", launch.getStatus());

        // Step 1: Complete the reverse path
        if (pathStarted && !pathComplete) {
            if (!follower.isBusy()) {
                pathComplete = true;
                follower.startTeleopDrive(true);
                follower.setTeleOpDrive(0, 0, 0, true); // Stop robot
                launchStartTime = System.currentTimeMillis();
            }
        }

        // Step 2: Launch the ball from slot 1 (index 0) with double pulse
        if (pathComplete && !launchInitiated) {
            // Move to launch slot 1 (index 0)
            //indexer.moveToLaunchSlot(0);
            
            // Start flywheel spin-up
            flywheel.holdConstantSpeed();
            flywheelSpinUpStartTime = System.currentTimeMillis();
            launchInitiated = true;
        }

        // Step 3: Update indexer subsystem and handle double pulse
        if (launchInitiated) {
            indexer.update();

            if (!flywheelStopped) {
                flywheel.holdConstantSpeed();
            }
            
            // Wait for flywheel to spin up before firing lever
            long spinUpElapsed = System.currentTimeMillis() - flywheelSpinUpStartTime;
            if (leverPulseCount == 0 && spinUpElapsed >= FLYWHEEL_SPIN_UP_TIME_MS) {
                // First lever pulse after spin-up delay
                indexer.handleLeverButton(true);
                leverPulseCount = 1;
                telemetry.addData("Status", "Flywheel ready, first pulse triggered");
            }
            
            // Check if first pulse is done and trigger second pulse
            if (leverPulseCount == 1 && !indexer.getLeverState() && !firstPulseDone) {
                firstPulseDone = true;
                firstPulseCompleteTime = System.currentTimeMillis();
                telemetry.addData("Status", "First pulse complete, waiting for second...");
            }
            
            // Trigger second pulse after delay
            if (firstPulseDone && System.currentTimeMillis() - firstPulseCompleteTime >= PULSE_DELAY_MS) {
                indexer.handleLeverButton(true);
                leverPulseCount = 2;
                telemetry.addData("Status", "Second pulse triggered");
            }
            
            // Check if second pulse is done and keep flywheel spinning briefly
            if (leverPulseCount == 2 && !indexer.getLeverState() && !flywheelStopped) {
                long postLaunchTime = System.currentTimeMillis() - firstPulseCompleteTime;
                if (postLaunchTime >= PULSE_DELAY_MS + POST_LAUNCH_SPIN_TIME_MS) {
                    telemetry.addData("Status", "Launch Complete");
                    // Stop flywheel after post-launch spin time
                    flywheel.setPower(0);
                    flywheelStopped = true;
                    launchComplete = true;
                }
            }
            
            // Safety timeout
            if (System.currentTimeMillis() - launchStartTime > LAUNCH_TIMEOUT_MS) {
                indexer.handleLeverButton(false);
                if (!flywheelStopped) {
                    flywheel.setPower(0);
                    flywheelStopped = true;
                }
                launchComplete = true;
            }
        }

        // Step 4: Move 2 feet to the right after launch completes
        if (launchComplete && !lateralMoveStarted) {
            Pose currentPose = follower.getPose();
            // Move 2 feet (24 inches) to the right (negative Y direction)
            lateralMove = new Path(new BezierLine(currentPose, new Pose(currentPose.getX(), currentPose.getY() - 24, currentPose.getHeading())));
            lateralMove.setConstantHeadingInterpolation(currentPose.getHeading());
            follower.followPath(lateralMove);
            lateralMoveStarted = true;
            telemetry.addData("Status", "Moving right 2 feet");
        }

        // Step 5: Complete lateral movement
        if (lateralMoveStarted && !lateralMoveComplete) {
            if (!follower.isBusy()) {
                lateralMoveComplete = true;
                follower.startTeleopDrive(true);
                follower.setTeleOpDrive(0, 0, 0, true);
                telemetry.addData("Status", "Auto Complete");
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (indexer != null) {
            indexer.handleLeverButton(false);
        }
        if (flywheel != null) {
            flywheel.setPower(0);
        }
    }
}

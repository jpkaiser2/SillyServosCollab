package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private final List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    // ===== Fixed target ID RED (locked) =====
    private int targetTagId = 24;

    private boolean initialized = false;

    private static final Size DEFAULT_RES = new Size(640, 480);

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        init(hwMap, telemetry, "Webcam", DEFAULT_RES);
    }

    public void init(HardwareMap hwMap, Telemetry telemetry, String webcamName, Size resolution) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, webcamName))
                .setCameraResolution(resolution)
                .addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
        initialized = true;
    }

    /** Set the only tag ID the system is allowed to track (locked target). */
    public void setTargetTagId(int id) {
        this.targetTagId = id;
    }

    /** Returns the locked target tag ID. */
    public int getTargetTagId() {
        return targetTagId;
    }

    /** Call every loop. Copies detections into a stable internal list. */
    public void update() {
        if (!initialized || aprilTagProcessor == null) return;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        detectedTags.clear();
        if (detections != null) detectedTags.addAll(detections);
    }

    /** Immutable view (do not modify). */
    public List<AprilTagDetection> getDetectedTags() {
        return Collections.unmodifiableList(detectedTags);
    }

    /** Returns the detection for the locked target ID (default 21), or null if not visible. */
    public AprilTagDetection getTargetTag() {
        return getTagById(targetTagId);
    }

    /** Returns the detection for a specific ID, or null. */
    public AprilTagDetection getTagById(int id) {
        for (AprilTagDetection d : detectedTags) {
            if (d != null && d.id == id) return d;
        }
        return null;
    }

    /**
     * Pixel->bearing fallback if ftcPose is null.
     * bearingDeg ≈ ((cx - imageCenterX) / imageCenterX) * (HFOV/2)
     */
    public static double estimateBearingFromPixels(double centerXpx, int imageWidthPx, double hfovDeg) {
        double cx = imageWidthPx * 0.5;
        double norm = (centerXpx - cx) / cx; // -1..1
        return norm * (hfovDeg * 0.5);
    }

    /** Telemetry for a single detection. */
    public void displayDetectionTelemetry(AprilTagDetection d) {
        if (telemetry == null || d == null) return;

        telemetry.addData("TargetID", targetTagId);
        telemetry.addData("SeenID", d.id);

        if (d.metadata != null) telemetry.addData("TagName", d.metadata.name);

        if (d.ftcPose != null) {
            telemetry.addData("Range(in)", String.format("%.1f", d.ftcPose.range));
            telemetry.addData("Bearing(deg)", String.format("%.1f", d.ftcPose.bearing));
            telemetry.addData("Elev(deg)", String.format("%.1f", d.ftcPose.elevation));
            telemetry.addData("XYZ(in)", String.format("%.1f %.1f %.1f", d.ftcPose.x, d.ftcPose.y, d.ftcPose.z));
            telemetry.addData("PRY(deg)", String.format("%.1f %.1f %.1f", d.ftcPose.pitch, d.ftcPose.roll, d.ftcPose.yaw));
        } else {
            telemetry.addData("Center(px)", String.format("%.0f, %.0f", d.center.x, d.center.y));
        }
    }

    public void setStreaming(boolean enabled) {
        if (visionPortal == null) return;
        if (enabled) visionPortal.resumeStreaming();
        else visionPortal.stopStreaming();
    }

    public boolean isStreaming() {
        if (visionPortal == null) return false;
        return visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public void stop() {
        initialized = false;
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        aprilTagProcessor = null;
        detectedTags.clear();
    }
}

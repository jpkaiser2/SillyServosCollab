package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CheckPatternSubsystem {

    private enum State {
        IDLE, MOVE, SETTLE, SAMPLE, BLACK_NUDGE, DONE
    }

    private final IndexerSubsystem indexer;
    private final NormalizedColorSensor colorSensor;

    private boolean checking = false;
    private State state = State.IDLE;

    // TeleOp expects these strings
    // possible values: "empty", "green", "purple", "unknown"
    private final String[] indexerPattern = new String[] { "empty", "empty", "empty" };

    // slot 0=Launch1, 1=Launch2, 2=Launch3
    private int slotIndex = 0;
    private int blackRetry = 0;

    private final ElapsedTime timer = new ElapsedTime();

    // ---------- TUNABLES ----------
    public long settleMs = 140;
    public int samplesPerRead = 7;
    public long sampleSpacingMs = 15;

    public float greenHue = 140f;
    public float purpleHue = 250f;
    public float hueTol = 25f;

    public float blackValueMax = 0.10f;
    public float blackSaturationMax = 0.20f;

    public int maxBlackRetries = 3;
    public int nudgeTicks = 6;
    public long nudgeSettleMs = 90;
    // -----------------------------

    // sampling accumulators
    private int sampleCount = 0;
    private float hueSum = 0f, satSum = 0f, valSum = 0f;

    // for telemetry
    private float[] lastHsv = new float[] { 0f, 0f, 0f };

    public CheckPatternSubsystem(HardwareMap hardwareMap, IndexerSubsystem indexer, String colorSensorName) {
        this.indexer = indexer;
        this.colorSensor = hardwareMap.get(NormalizedColorSensor.class, colorSensorName);
    }

    public void start() {
        checking = true;
        state = State.MOVE;
        slotIndex = 0;
        blackRetry = 0;
        indexerPattern[0] = "empty";
        indexerPattern[1] = "empty";
        indexerPattern[2] = "empty";
        timer.reset();
    }

    public void stop() {
        checking = false;
        state = State.IDLE;
    }

    public boolean getStatus() {
        return checking;
    }

    public String[] getIndexerPattern() {
        return indexerPattern;
    }

    // For telemetry: returns most recent HSV reading (not averaged).
    public float[] getHsv() {
        readHSVInto(lastHsv);
        return lastHsv;
    }

    public void update() {
        if (!checking) {
            state = State.IDLE;
            return;
        }

        switch (state) {
            case MOVE: {
                commandMove(slotIndex);
                state = State.SETTLE;
                timer.reset();
                break;
            }

            case SETTLE: {
                long requiredSettle;
                if (blackRetry > 0) {
                    requiredSettle = nudgeSettleMs;
                } else {
                    requiredSettle = settleMs;
                }
                if (!indexer.isMoving() && timer.milliseconds() >= requiredSettle) {
                    beginSampling();
                    state = State.SAMPLE;
                    timer.reset();
                }
                break;
            }

            case SAMPLE: {
                if (timer.milliseconds() >= sampleSpacingMs) {
                    timer.reset();
                    accumulateSample();

                    if (sampleCount >= samplesPerRead) {
                        String result = classifyAverage();

                        if ("empty".equals(result)) {
                            // could be hole/background OR truly empty
                            if (blackRetry < maxBlackRetries) {
                                blackRetry++;
                                state = State.BLACK_NUDGE;
                            } else {
                                indexerPattern[slotIndex] = "empty";
                                advance();
                            }
                        } else {
                            indexerPattern[slotIndex] = result;
                            advance();
                        }
                    }
                }
                break;
            }

            case BLACK_NUDGE: {
                // alternate direction each retry
                int dir;
                if (blackRetry % 2 == 1) {
                    dir = +1;
                } else {
                    dir = -1;
                }
                indexer.nudgeTicks(dir * nudgeTicks);
                state = State.SETTLE;
                timer.reset();
                break;
            }

            case DONE: {
                checking = false;
                state = State.IDLE;
                break;
            }

            case IDLE:
            default:
                break;
        }
    }

    // ---------------- helpers ----------------

    private void commandMove(int slot) {
        indexer.moveToLaunchSlot(slot);
    }

    private void beginSampling() {
        sampleCount = 0;
        hueSum = 0f;
        satSum = 0f;
        valSum = 0f;
    }

    private void accumulateSample() {
        float[] hsv = new float[3];
        readHSVInto(hsv);
        hueSum += hsv[0];
        satSum += hsv[1];
        valSum += hsv[2];
        sampleCount++;
        lastHsv = hsv;
    }

    private String classifyAverage() {
        float avgHue = hueSum / Math.max(1, sampleCount);
        float avgSat = satSum / Math.max(1, sampleCount);
        float avgVal = valSum / Math.max(1, sampleCount);

        boolean looksBlack = (avgVal <= blackValueMax) || (avgSat <= blackSaturationMax);
        if (looksBlack)
            return "empty";

        if (isHueNear(avgHue, greenHue, hueTol))
            return "green";
        if (isHueNear(avgHue, purpleHue, purpleHueTol()))
            return "purple";

        return "unknown";
    }

    // If purple tends to be noisier, need to slightly widen it without changing
    // green.
    private float purpleHueTol() {
        return hueTol;
    }

    private boolean isHueNear(float hue, float target, float tol) {
        float d = Math.abs(hue - target);
        d = Math.min(d, 360f - d);
        return d <= tol;
    }

    private void readHSVInto(float[] hsvOut) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvOut);
    }

    private void advance() {
        // reset black retry for next slot
        blackRetry = 0;

        slotIndex++;
        if (slotIndex >= 3) {
            state = State.DONE;
        } else {
            state = State.MOVE;
        }
        timer.reset();
    }
}

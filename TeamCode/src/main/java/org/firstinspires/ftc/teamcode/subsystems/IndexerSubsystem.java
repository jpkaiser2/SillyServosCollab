package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class IndexerSubsystem {

    @IgnoreConfigurable
    private final DcMotorEx indexerMotor;
    @IgnoreConfigurable
    private final Servo feedLeverServo;
    @IgnoreConfigurable
    private final TouchSensor magnetSensor;

    // ---------------------------------
    // PHASE / MAGNET HOMING (TUNABLE)
    // ---------------------------------


    /** Manually drive indexer motor (for tuning). */
    public void setManualPower(double power) {
        indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexerMotor.setPower(power);
    }

    /** Stop manual control and hold position. */
    public void stopManual() {
        indexerMotor.setPower(0.0);
        indexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /** Enable/disable magnet correction live while tuning. */
    public static boolean MAGNET_CORRECTION_ENABLED = true;

    /**
     * Encoder ticks per full revolution of the indexer output (after gearing).
     * Tune this first. Example: rotate exactly 1 full turn and read delta encoder.
     */
    public static int TICKS_PER_REV = 288;

    /**
     * The phase (0..TICKS_PER_REV-1) where Magnet A is centered on the hall sensor.
     * Tune by rotating slowly and averaging rise/fall center.
     */
    public static int MAG_A_PHASE = 67;

    /**
     * The phase (0..TICKS_PER_REV-1) where Magnet B is centered on the hall sensor.
     * Should be about 120 degrees from A (≈ TICKS_PER_REV/3).
     */
    public static int MAG_B_PHASE = 172;

    /** How close (ticks) we must be to a magnet phase to accept it as that magnet. */
    public static int MAGNET_WINDOW_TICKS = 50;

    /** Debounce between magnet detections so one pass doesn't trigger multiple times. */
    public static long MAGNET_DEBOUNCE_MS = 180;

    /** Use rising edge (pressed becomes true) as the detection edge. */
    public static boolean USE_RISING_EDGE = true;

    /**
     * The continuously-updated offset such that:
     *   virtualPhase = wrap(rawEncoder + phaseOffsetTicks)
     * Magnets update this offset to prevent drift.
     */
    public static int phaseOffsetTicks = 0;

    // Debug / tuning telemetry fields
    public static int dbgRawEncoder = 0;
    public static int dbgRawPhase = 0;
    public static int dbgVirtualPhase = 0;
    public static int dbgLastMagnetPhase = 0;   // expected phase (A or B)
    public static int dbgLastSnapError = 0;     // signed ticks adjusted on last snap
    public static String dbgLastMagnetName = "none";
    public static boolean dbgMagnetPressed = false;

    // Internal edge detection
    private boolean prevMagnetPressed = false;
    private final ElapsedTime magnetDebounceTimer = new ElapsedTime();

    // ---------------------------------
    // SLOT PHASES (TUNABLE)
    // IMPORTANT: these are PHASES now, not absolute encoder ticks.
    // ---------------------------------

    public static int LAUNCH_1 = 3;
    public static int LAUNCH_2 = 194;
    public static int LAUNCH_3 = 91;

    public static int INTAKE_1 = 140;
    public static int INTAKE_2 = 30;
    public static int INTAKE_3 = 220;

    // ---------------------------------
    // Lever pulse config
    // ---------------------------------

    @IgnoreConfigurable
    private final ElapsedTime leverTimer = new ElapsedTime();
    @IgnoreConfigurable
    private boolean leverPulsing = false;
    @IgnoreConfigurable
    private long leverPulseMs = 200;
    @IgnoreConfigurable
    private double leverIdlePos = 0.2;
    @IgnoreConfigurable
    private double leverEngagedPos = 0.8;
    @IgnoreConfigurable
    private double leverMaxPos = 0.6;

    public IndexerSubsystem(HardwareMap hardwareMap, String indexerMotorName, String feedLeverServoName) {
        this.indexerMotor = hardwareMap.get(DcMotorEx.class, indexerMotorName);
        this.indexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.indexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.indexerMotor.setPower(0.0);

        this.feedLeverServo = hardwareMap.get(Servo.class, feedLeverServoName);
        this.feedLeverServo.setDirection(Servo.Direction.REVERSE);
        this.feedLeverServo.setPosition(Math.min(leverIdlePos, leverMaxPos));

        this.magnetSensor = hardwareMap.get(TouchSensor.class, "magnet");

        // initialize edge state + debounce timer
        prevMagnetPressed = magnetSensor.isPressed();
        magnetDebounceTimer.reset();
    }

    /** Call once per loop. */
    public void update() {
        updateLever();
        updateMagnetCorrection();
        updateDebug();
    }

    // ---------------------------------
    // MAGNET CORRECTION CORE
    // ---------------------------------

    private void updateMagnetCorrection() {
        if (!MAGNET_CORRECTION_ENABLED) {
            prevMagnetPressed = magnetSensor.isPressed();
            return;
        }

        boolean pressed = magnetSensor.isPressed();
        boolean rising = (!prevMagnetPressed && pressed);
        boolean falling = (prevMagnetPressed && !pressed);

        boolean edge;
        if (USE_RISING_EDGE) edge = rising;
        else edge = falling;

        if (edge) {
            // debounce
            if (magnetDebounceTimer.milliseconds() >= MAGNET_DEBOUNCE_MS) {
                magnetDebounceTimer.reset();
                snapOffsetToNearestMagnet();
            }
        }

        prevMagnetPressed = pressed;
    }

    /**
     * Snap the phaseOffsetTicks so the current raw encoder maps to the closest magnet phase.
     * This is the key: we only update ONE offset, never "add offsets into presets".
     */
    private void snapOffsetToNearestMagnet() {
        if (TICKS_PER_REV <= 0) return;

        int raw = indexerMotor.getCurrentPosition();
        int rawPhase = wrap(raw, TICKS_PER_REV);

        // choose which magnet we are seeing by distance in phase-space
        int distA = Math.abs(shortestSignedDelta(rawPhase, MAG_A_PHASE, TICKS_PER_REV));
        int distB = Math.abs(shortestSignedDelta(rawPhase, MAG_B_PHASE, TICKS_PER_REV));

        int expectedPhase;
        String magnetName;

        if (distA <= distB) {
            expectedPhase = MAG_A_PHASE;
            magnetName = "A";
        } else {
            expectedPhase = MAG_B_PHASE;
            magnetName = "B";
        }

        // only accept if close enough to a magnet phase
        int acceptDist;
        if (distA <= distB) acceptDist = distA; else acceptDist = distB;

        if (acceptDist > MAGNET_WINDOW_TICKS) {
            dbgLastMagnetName = "reject";
            return;
        }

        // Desired offset (wrapped) so that wrap(raw + offset) == expectedPhase
        int desiredOffsetWrapped = wrap(expectedPhase - rawPhase, TICKS_PER_REV);
        int currentOffsetWrapped = wrap(phaseOffsetTicks, TICKS_PER_REV);

        // Adjust offset by the shortest signed delta between currentOffsetWrapped and desiredOffsetWrapped
        int delta = shortestSignedDelta(currentOffsetWrapped, desiredOffsetWrapped, TICKS_PER_REV);
        phaseOffsetTicks += delta;

        dbgLastMagnetName = magnetName;
        dbgLastMagnetPhase = expectedPhase;
        dbgLastSnapError = delta;
    }

    private void updateDebug() {
        dbgRawEncoder = indexerMotor.getCurrentPosition();
        if (TICKS_PER_REV > 0) {
            dbgRawPhase = wrap(dbgRawEncoder, TICKS_PER_REV);
            dbgVirtualPhase = wrap(dbgRawEncoder + phaseOffsetTicks, TICKS_PER_REV);
        } else {
            dbgRawPhase = 0;
            dbgVirtualPhase = 0;
        }
        dbgMagnetPressed = magnetSensor.isPressed();
    }

    // ---------------------------------
    // PHASE-BASED POSITIONING
    // ---------------------------------

    public void setIndexerPosition(String inputPosition) {
        int phase;
        if ("Launch1".equals(inputPosition)) phase = LAUNCH_1;
        else if ("Launch2".equals(inputPosition)) phase = LAUNCH_2;
        else if ("Launch3".equals(inputPosition)) phase = LAUNCH_3;
        else if ("Intake1".equals(inputPosition)) phase = INTAKE_1;
        else if ("Intake2".equals(inputPosition)) phase = INTAKE_2;
        else if ("Intake3".equals(inputPosition)) phase = INTAKE_3;
        else return;

        setTargetPhase(phase);
    }

    /**
     * Move to a desired PHASE (0..TICKS_PER_REV-1), choosing the nearest absolute target.
     */
    public void setTargetPhase(int desiredPhase) {
        if (TICKS_PER_REV <= 0) return;

        desiredPhase = wrap(desiredPhase, TICKS_PER_REV);

        int raw = indexerMotor.getCurrentPosition();

        // virtual absolute position
        int vAbs = raw + phaseOffsetTicks;

        // find nearest k*rev + desiredPhase to current vAbs
        int curRev = floorDiv(vAbs, TICKS_PER_REV);
        int cand1 = curRev * TICKS_PER_REV + desiredPhase;
        int cand2 = (curRev + 1) * TICKS_PER_REV + desiredPhase;
        int cand0 = (curRev - 1) * TICKS_PER_REV + desiredPhase;

        int best = cand1;
        if (Math.abs(cand2 - vAbs) < Math.abs(best - vAbs)) best = cand2;
        if (Math.abs(cand0 - vAbs) < Math.abs(best - vAbs)) best = cand0;

        // convert back to raw target
        int rawTarget = best - phaseOffsetTicks;

        indexerMotor.setTargetPosition(rawTarget);
        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexerMotor.setPower(0.6);
    }

    /** Nudge the indexer target by a number of encoder ticks (raw ticks). */
    public void nudgeTicks(int deltaTicks) {
        int base;
        if (indexerMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) base = indexerMotor.getTargetPosition();
        else base = indexerMotor.getCurrentPosition();

        int target = base + deltaTicks;
        indexerMotor.setTargetPosition(target);

        try { indexerMotor.setTargetPositionTolerance(1); } catch (Exception ignore) {}

        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexerMotor.setPower(0.6);
    }

    // ---------------------------------
    // LEVER
    // ---------------------------------

    public void handleLeverButton(boolean pressed) {
        if (pressed && !leverPulsing) {
            leverPulsing = true;
            leverTimer.reset();
            feedLeverServo.setPosition(Math.min(leverEngagedPos, leverMaxPos));
        }
    }

    public void updateLever() {
        if (leverPulsing && leverTimer.milliseconds() >= leverPulseMs) {
            feedLeverServo.setPosition(Math.min(leverIdlePos, leverMaxPos));
            leverPulsing = false;
        }
    }

    public boolean getLeverState() {
        return leverPulsing;
    }

    public void setLeverConfig(long pulseMs, double idle, double engaged) {
        leverPulseMs = pulseMs;
        leverIdlePos = Math.min(idle, leverMaxPos);
        leverEngagedPos = Math.min(engaged, leverMaxPos);
    }

    public void setLeverMax(double max) {
        leverMaxPos = Math.max(0.0, Math.min(1.0, max));
        leverIdlePos = Math.min(leverIdlePos, leverMaxPos);
        leverEngagedPos = Math.min(leverEngagedPos, leverMaxPos);
        if (!leverPulsing) feedLeverServo.setPosition(leverIdlePos);
    }

    // ---------------------------------
    // STATUS / HELPERS
    // ---------------------------------

    public boolean isMoving() {
        return indexerMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && indexerMotor.isBusy();
    }

    public int getCurrentPosition() {
        return indexerMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return indexerMotor.getTargetPosition();
    }

    public boolean getMagnetState() {
        return dbgMagnetPressed;
    }

    public String getStatus() {
        boolean busy = indexerMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && indexerMotor.isBusy();
        return String.format(
                "mode=%s busy=%s raw=%d rawPhase=%d vPhase=%d off=%d lastMag=%s snap=%d",
                indexerMotor.getMode(),
                busy,
                dbgRawEncoder,
                dbgRawPhase,
                dbgVirtualPhase,
                phaseOffsetTicks,
                dbgLastMagnetName,
                dbgLastSnapError
        );
    }

    // --- math helpers ---

    private static int wrap(int x, int mod) {
        if (mod <= 0) return 0;
        int r = x % mod;
        if (r < 0) r += mod;
        return r;
    }

    /**
     * Signed shortest delta from a -> b on a circle of size mod.
     * Result is in (-mod/2 .. +mod/2].
     */
    private static int shortestSignedDelta(int a, int b, int mod) {
        int d = wrap(b - a, mod);
        if (d > mod / 2) d -= mod;
        return d;
    }

    private static int floorDiv(int a, int b) {
        // Java truncates; need floor division
        int q = a / b;
        int r = a % b;
        if (r != 0 && ((r < 0) != (b < 0))) q--;
        return q;
    }
}

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

    // ----------------------------
    // Hardware
    // ----------------------------

    @IgnoreConfigurable
    private final DcMotorEx indexerMotor;
    @IgnoreConfigurable
    private final Servo feedLeverServo;
    @IgnoreConfigurable
    private final TouchSensor magnetSensor;

    // ----------------------------
    // Manual tuning mode (raw power)
    // ----------------------------

    @IgnoreConfigurable
    private boolean manualMode = false;
    @IgnoreConfigurable
    private double manualPower = 0.0;

    /** Manually drive indexer motor (for tuning). */
    public void setManualPower(double power) {
        manualMode = true;
        manualPower = clamp(power, -1.0, 1.0);
        motionState = MotionState.IDLE; // stop phase motion
        indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexerMotor.setPower(manualPower);
    }

    /** Stop manual control. */
    public void stopManual() {
        manualMode = false;
        manualPower = 0.0;
        indexerMotor.setPower(0.0);
        // Keep RUN_WITHOUT_ENCODER + BRAKE, so it holds reasonably.
        indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isManualMode() {
        return manualMode;
    }

    // ----------------------------
    // PHASE / MAGNET HOMING (TUNABLE)
    // ----------------------------

    /** Enable/disable magnet correction live while tuning. */
    public static boolean MAGNET_CORRECTION_ENABLED = true;

    /**
     * Encoder ticks per full revolution of the indexer output (direct-drive => measure this!).
     * Tune FIRST.
     */
    public static int TICKS_PER_REV = 288;

    /** Magnet A center phase (0..TICKS_PER_REV-1). */
    public static int MAG_A_PHASE = 70;

    /** Magnet B center phase (0..TICKS_PER_REV-1). */
    public static int MAG_B_PHASE = 168;

    /** Accept magnet if within this many phase ticks. */
    public static int MAGNET_WINDOW_TICKS = 75;

    /** Debounce between magnet detections so one pass doesn't trigger multiple times. */
    public static long MAGNET_DEBOUNCE_MS = 200;

    /** Use rising edge as the detection edge. */
    public static boolean USE_RISING_EDGE = true;

    /**
     * Offset so that:
     *   virtualPhase = wrap(rawEncoder + phaseOffsetTicks)
     * Magnets update this offset to prevent drift.
     */
    public static int phaseOffsetTicks = 0;

    // ----------------------------
    // SLOT PHASES (TUNABLE)
    // ----------------------------

    // IMPORTANT: these are PHASES (0..TICKS_PER_REV-1), not raw encoder ticks
    public static int LAUNCH_1 = 0;
    public static int LAUNCH_2 = 186;
    public static int LAUNCH_3 = 79;

    public static int INTAKE_1 = 141;
    public static int INTAKE_2 = 33;
    public static int INTAKE_3 = 235;

    // ----------------------------
    // P-Controller Motion (NO RUN_TO_POSITION)
    // ----------------------------

    /** Proportional gain in power-per-tick. Start: 0.006..0.02. */
    public static double kP = 0.008;

    /** Max motor power for phase moves. */
    public static double maxPower = 0.35;

    /** Minimum power to overcome stiction when far from target. */
    public static double minPower = 0.18;

    /** If |error| <= deadbandTicks, consider "arrived". */
    public static int deadbandTicks = 6;

    /** Require error to stay in deadband for this long before finishing. */
    public static long settleMs = 80;

    /** Safety timeout for any move. */
    public static long moveTimeoutMs = 1400;

    /** If error is small, allow power to drop below minPower (prevents jitter). */
    public static int minPowerDisableWithinTicks = 20;

    // ----------------------------
    // Lever pulse config
    // ----------------------------

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

    // ----------------------------
    // Magnet edge detection (internal)
    // ----------------------------

    @IgnoreConfigurable
    private boolean prevMagnetPressed = false;
    @IgnoreConfigurable
    private final ElapsedTime magnetDebounceTimer = new ElapsedTime();

    // ----------------------------
    // Motion state (internal)
    // ----------------------------

    private enum MotionState { IDLE, MOVING_PHASE }

    @IgnoreConfigurable
    private MotionState motionState = MotionState.IDLE;

    /** Target phase we want to reach (0..TICKS_PER_REV-1). */
    @IgnoreConfigurable
    private int targetPhase = 0;

    @IgnoreConfigurable
    private final ElapsedTime moveTimer = new ElapsedTime();
    @IgnoreConfigurable
    private final ElapsedTime settleTimer = new ElapsedTime();
    @IgnoreConfigurable
    private boolean inDeadband = false;

    // ----------------------------
    // Debug (internal, exposed via getters)
    // ----------------------------

    private int dbgRawEncoder = 0;
    private int dbgRawPhase = 0;
    private int dbgVirtualPhase = 0;
    private int dbgError = 0;

    private int dbgLastMagnetPhase = 0;
    private int dbgLastSnapError = 0; // <-- FIX 2: not static, accessed via getter
    private String dbgLastMagnetName = "none";
    private boolean dbgMagnetPressed = false;

    // ----------------------------
    // Constructor
    // ----------------------------

    public IndexerSubsystem(HardwareMap hardwareMap, String indexerMotorName, String feedLeverServoName) {
        this.indexerMotor = hardwareMap.get(DcMotorEx.class, indexerMotorName);
        this.indexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.indexerMotor.setPower(0.0);

        this.feedLeverServo = hardwareMap.get(Servo.class, feedLeverServoName);
        this.feedLeverServo.setDirection(Servo.Direction.REVERSE);
        this.feedLeverServo.setPosition(Math.min(leverIdlePos, leverMaxPos));

        this.magnetSensor = hardwareMap.get(TouchSensor.class, "magnet");
        this.prevMagnetPressed = magnetSensor.isPressed();
        this.magnetDebounceTimer.reset();
    }

    // ----------------------------
    // Public API (called from TeleOp / other subsystems)
    // ----------------------------

    /** Call once per loop. */
    public void update() {
        updateLever();
        updateMagnetCorrection();
        updateMotion();
        updateDebug();
    }

    /** TeleOp presets call this. */
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

    /** Set a desired phase (0..TICKS_PER_REV-1). */
    public void setTargetPhase(int desiredPhase) {
        if (TICKS_PER_REV <= 0) return;

        desiredPhase = wrap(desiredPhase, TICKS_PER_REV);

        manualMode = false;
        manualPower = 0.0;

        targetPhase = desiredPhase;
        motionState = MotionState.MOVING_PHASE;
        moveTimer.reset();
        settleTimer.reset();
        inDeadband = false;

        indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Nudge the target by phase ticks (used by CheckPatternSubsystem hole-avoidance). */
    public void nudgeTicks(int deltaTicks) {
        if (TICKS_PER_REV <= 0) return;
        setTargetPhase(targetPhase + deltaTicks);
    }

    /** Whether indexer is actively moving to a target (manual or phase). */
    public boolean isMoving() {
        if (manualMode) {
            return Math.abs(manualPower) > 0.01;
        }
        return motionState == MotionState.MOVING_PHASE;
    }

    public int getCurrentPosition() {
        return indexerMotor.getCurrentPosition();
    }

    /** For compatibility: returns a best-guess raw target corresponding to targetPhase near current position. */
    public int getTargetPosition() {
        if (TICKS_PER_REV <= 0) return indexerMotor.getCurrentPosition();

        int raw = indexerMotor.getCurrentPosition();
        int vAbs = raw + phaseOffsetTicks;
        int curRev = floorDiv(vAbs, TICKS_PER_REV);

        int dp = wrap(targetPhase, TICKS_PER_REV);

        int cand1 = curRev * TICKS_PER_REV + dp;
        int cand2 = (curRev + 1) * TICKS_PER_REV + dp;
        int cand0 = (curRev - 1) * TICKS_PER_REV + dp;

        int best = cand1;
        if (Math.abs(cand2 - vAbs) < Math.abs(best - vAbs)) best = cand2;
        if (Math.abs(cand0 - vAbs) < Math.abs(best - vAbs)) best = cand0;

        return best - phaseOffsetTicks;
    }

    // ----------------------------
    // Lever
    // ----------------------------

    public void handleLeverButton(boolean pressed) {
        if (pressed && !leverPulsing) {
            leverPulsing = true;
            leverTimer.reset();
            feedLeverServo.setPosition(Math.min(leverEngagedPos, leverMaxPos));
        }
    }

    private void updateLever() {
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

    // ----------------------------
    // Magnet correction
    // ----------------------------

    private void updateMagnetCorrection() {
        if (!MAGNET_CORRECTION_ENABLED) {
            prevMagnetPressed = magnetSensor.isPressed();
            return;
        }

        boolean pressed = magnetSensor.isPressed();
        boolean rising = (!prevMagnetPressed && pressed);
        boolean falling = (prevMagnetPressed && !pressed);

        boolean edge = false;
        if (USE_RISING_EDGE) {
            if (rising) edge = true;
        } else {
            if (falling) edge = true;
        }

        if (edge) {
            if (magnetDebounceTimer.milliseconds() >= MAGNET_DEBOUNCE_MS) {
                magnetDebounceTimer.reset();
                snapOffsetToNearestMagnet();
            }
        }

        prevMagnetPressed = pressed;
    }

    private void snapOffsetToNearestMagnet() {
        if (TICKS_PER_REV <= 0) return;

        int raw = indexerMotor.getCurrentPosition();
        int rawPhase = wrap(raw, TICKS_PER_REV);

        int distA = Math.abs(shortestSignedDelta(rawPhase, wrap(MAG_A_PHASE, TICKS_PER_REV), TICKS_PER_REV));
        int distB = Math.abs(shortestSignedDelta(rawPhase, wrap(MAG_B_PHASE, TICKS_PER_REV), TICKS_PER_REV));

        int expectedPhase;
        String name;
        int acceptDist;

        if (distA <= distB) {
            expectedPhase = wrap(MAG_A_PHASE, TICKS_PER_REV);
            name = "A";
            acceptDist = distA;
        } else {
            expectedPhase = wrap(MAG_B_PHASE, TICKS_PER_REV);
            name = "B";
            acceptDist = distB;
        }

        if (acceptDist > MAGNET_WINDOW_TICKS) {
            dbgLastMagnetName = "reject";
            return;
        }

        int desiredOffsetWrapped = wrap(expectedPhase - rawPhase, TICKS_PER_REV);
        int currentOffsetWrapped = wrap(phaseOffsetTicks, TICKS_PER_REV);

        int delta = shortestSignedDelta(currentOffsetWrapped, desiredOffsetWrapped, TICKS_PER_REV);
        phaseOffsetTicks += delta;

        dbgLastMagnetName = name;
        dbgLastMagnetPhase = expectedPhase;
        dbgLastSnapError = delta; // <-- FIX 2 stored here
    }

    // ----------------------------
    // Motion (P-controller on phase)
    // ----------------------------

    private void updateMotion() {
        if (manualMode) {
            // manual owns motor power
            return;
        }

        if (motionState == MotionState.IDLE) {
            indexerMotor.setPower(0.0);
            return;
        }

        if (motionState == MotionState.MOVING_PHASE) {
            if (TICKS_PER_REV <= 0) {
                indexerMotor.setPower(0.0);
                motionState = MotionState.IDLE;
                return;
            }

            if (moveTimer.milliseconds() >= moveTimeoutMs) {
                indexerMotor.setPower(0.0);
                motionState = MotionState.IDLE;
                return;
            }

            int curVPhase = getVirtualPhase();
            int tgt = wrap(targetPhase, TICKS_PER_REV);

            int err = shortestSignedDelta(curVPhase, tgt, TICKS_PER_REV);
            dbgError = err;

            int absErr = Math.abs(err);

            if (absErr <= deadbandTicks) {
                if (!inDeadband) {
                    inDeadband = true;
                    settleTimer.reset();
                }
                indexerMotor.setPower(0.0);

                if (settleTimer.milliseconds() >= settleMs) {
                    motionState = MotionState.IDLE;
                    inDeadband = false;
                }
                return;
            } else {
                inDeadband = false;
            }

            double pwr = kP * (double) err;

            if (pwr > maxPower) pwr = maxPower;
            if (pwr < -maxPower) pwr = -maxPower;

            if (absErr > minPowerDisableWithinTicks) {
                if (pwr > 0.0 && pwr < minPower) pwr = minPower;
                if (pwr < 0.0 && pwr > -minPower) pwr = -minPower;
            }

            indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            indexerMotor.setPower(pwr);
        }
    }

    private int getVirtualPhase() {
        if (TICKS_PER_REV <= 0) return 0;
        int raw = indexerMotor.getCurrentPosition();
        return wrap(raw + phaseOffsetTicks, TICKS_PER_REV);
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

    // ----------------------------
    // Debug getters
    // ----------------------------

    public int getDbgRawPhase() {
        return dbgRawPhase;
    }

    public int getDbgVirtualPhase() {
        return dbgVirtualPhase;
    }

    public int getDbgPhaseOffsetTicks() {
        return phaseOffsetTicks;
    }

    public boolean getDbgMagnetPressed() {
        return dbgMagnetPressed;
    }

    public String getDbgLastMagnetName() {
        return dbgLastMagnetName;
    }

    public int getDbgLastSnapError() {
        return dbgLastSnapError;
    }

    public int getDbgError() {
        return dbgError;
    }

    // ----------------------------
    // Status / telemetry string
    // ----------------------------

    public String getStatus() {
        boolean busy = isMoving();
        return String.format(
                "mode=%s busy=%s raw=%d rawPhase=%d vPhase=%d off=%d tgtPhase=%d err=%d lastMag=%s snap=%d manual=%s",
                indexerMotor.getMode(),
                busy,
                dbgRawEncoder,
                dbgRawPhase,
                dbgVirtualPhase,
                phaseOffsetTicks,
                wrap(targetPhase, (TICKS_PER_REV <= 0 ? 1 : TICKS_PER_REV)),
                dbgError,
                dbgLastMagnetName,
                dbgLastSnapError,
                manualMode
        );
    }

    public boolean getMagnetState() {
        return dbgMagnetPressed;
    }

    // ----------------------------
    // Math helpers
    // ----------------------------

    private static int wrap(int x, int mod) {
        if (mod <= 0) return 0;
        int r = x % mod;
        if (r < 0) r += mod;
        return r;
    }

    /** Signed shortest delta from a -> b on a circle of size mod. */
    private static int shortestSignedDelta(int a, int b, int mod) {
        int d = wrap(b - a, mod);
        if (d > mod / 2) d -= mod;
        return d;
    }

    private static int floorDiv(int a, int b) {
        int q = a / b;
        int r = a % b;
        if (r != 0) {
            boolean rNeg = r < 0;
            boolean bNeg = b < 0;
            if (rNeg != bNeg) q--;
        }
        return q;
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}

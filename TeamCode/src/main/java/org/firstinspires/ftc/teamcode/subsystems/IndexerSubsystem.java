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
        motionState = MotionState.IDLE; // stop closed-loop motion
        indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexerMotor.setPower(manualPower);
    }

    /** Stop manual control (keeps BRAKE). */
    public void stopManual() {
        manualMode = false;
        manualPower = 0.0;
        indexerMotor.setPower(0.0);
        indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isManualMode() {
        return manualMode;
    }

    // ----------------------------
    // 3-MAGNET ABSOLUTE LAUNCH INDEXING (TUNABLE)
    // ----------------------------

    /** Enable/disable magnet correction. Leave ON for competition. */
    public static boolean MAGNET_CORRECTION_ENABLED = true;

    /**
     * Encoder ticks per full revolution of the indexer (direct drive).
     * REV Core Hex motor is 288 ticks/rev (at the motor shaft).
     * If you are truly direct-drive, this is usually correct: 288.
     */
    public static int TICKS_PER_REV = 288;

    /**
     * These are the PHASES (0..TICKS_PER_REV-1) where each LAUNCH magnet
     * is CENTERED on the hall sensor.
     *
     * Tune these by slowly rotating and recording the center phase for each magnet.
     */
    public static int MAG_LAUNCH_1_PHASE = 0;
    public static int MAG_LAUNCH_2_PHASE = 96;
    public static int MAG_LAUNCH_3_PHASE = 192;

    /** Accept a magnet only if the measured center is within this window of a known launch magnet phase. */
    public static int MAGNET_WINDOW_TICKS = 40;

    /** Debounce between detections so one pass doesn’t retrigger repeatedly. */
    public static long MAGNET_DEBOUNCE_MS = 140;

    /**
     * Phase offset so that:
     *   virtualPhase = wrap(rawEncoder + phaseOffsetTicks, TICKS_PER_REV)
     * Magnets update this offset to eliminate encoder drift/slop accumulation.
     */
    public static int phaseOffsetTicks = 0;

    // ----------------------------
    // Slot phases (tunable)
    // Launch phases are on magnets (ABSOLUTE).
    // Intake phases are between magnets (RELATIVE but stabilized by magnet offset snaps).
    // ----------------------------

    public static int LAUNCH_1 = 0;
    public static int LAUNCH_2 = 96;
    public static int LAUNCH_3 = 192;

    public static int INTAKE_1 = 159;
    public static int INTAKE_2 = 54;
    public static int INTAKE_3 = 250;

    // ----------------------------
    // P-controller Motion (NO RUN_TO_POSITION)
    // ----------------------------

    /** Proportional gain in power-per-tick. Typical start: 0.01..0.03 */
    public static double kP = 0.020;

    /** Maximum motor power for moves. */
    public static double maxPower = 0.45;

    /** Minimum power to overcome stiction when far from target. */
    public static double minPower = 0.18;

    /** If |error| <= deadbandTicks, consider "arrived". */
    public static int deadbandTicks = 3;

    /** Require error to stay in deadband for this long before finishing. */
    public static long settleMs = 90;

    /** Safety timeout for any move. */
    public static long moveTimeoutMs = 1500;

    /** Within this error, allow power to drop below minPower to prevent jitter. */
    public static int minPowerDisableWithinTicks = 18;

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
    // Magnet edge capture (internal)
    // We compute CENTER using rising+falling edges to reduce delay/jitter.
    // ----------------------------

    @IgnoreConfigurable
    private boolean prevMagnetPressed = false;
    @IgnoreConfigurable
    private final ElapsedTime magnetDebounceTimer = new ElapsedTime();

    @IgnoreConfigurable
    private boolean haveRisingEdge = false;
    @IgnoreConfigurable
    private int risingEdgeRaw = 0;

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
    // Debug fields (read via getters)
    // ----------------------------

    private int dbgRawEncoder = 0;
    private int dbgRawPhase = 0;
    private int dbgVirtualPhase = 0;
    private int dbgError = 0;

    private String dbgLastMagnetName = "none"; // "L1","L2","L3","reject"
    private int dbgLastMagnetPhase = 0;
    private int dbgLastSnapError = 0;

    private boolean dbgMagnetPressed = false;

    private int dbgRiseRaw = 0;
    private int dbgFallRaw = 0;
    private int dbgCenterRaw = 0;

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
    // Public API
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

    /** Nudge the target by phase ticks (used by CheckPatternSubsystem). */
    public void nudgeTicks(int deltaTicks) {
        if (TICKS_PER_REV <= 0) return;
        setTargetPhase(targetPhase + deltaTicks);
    }

    /** Whether indexer is actively moving to a target (manual or phase). */
    public boolean isMoving() {
        if (manualMode) {
            return Math.abs(manualPower) > 0.01;
        }
        if (motionState == MotionState.MOVING_PHASE) return true;
        return false;
    }

    public int getCurrentPosition() {
        return indexerMotor.getCurrentPosition();
    }

    /**
     * Compatibility helper: best-guess raw target corresponding to targetPhase near current position.
     * (TeleOp uses it only for telemetry; your motion uses P-control.)
     */
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
    // Magnet correction (3 magnets = 3 launch positions)
    // Uses RISE+FALL center to reduce delay/jitter.
    // ----------------------------

    private void updateMagnetCorrection() {
        if (!MAGNET_CORRECTION_ENABLED) {
            // keep edge detector in sync
            prevMagnetPressed = magnetSensor.isPressed();
            haveRisingEdge = false;
            return;
        }

        boolean pressed = magnetSensor.isPressed();
        boolean rising = (!prevMagnetPressed && pressed);
        boolean falling = (prevMagnetPressed && !pressed);

        if (rising) {
            haveRisingEdge = true;
            risingEdgeRaw = indexerMotor.getCurrentPosition();
            dbgRiseRaw = risingEdgeRaw;
        }

        if (falling) {
            int fallRaw = indexerMotor.getCurrentPosition();
            dbgFallRaw = fallRaw;

            if (haveRisingEdge) {
                haveRisingEdge = false;

                // debounce on FULL pulse (rise->fall)
                if (magnetDebounceTimer.milliseconds() >= MAGNET_DEBOUNCE_MS) {
                    magnetDebounceTimer.reset();

                    // center of the magnetic "pressed" window
                    int centerRaw = (int) Math.round((risingEdgeRaw + fallRaw) / 2.0);
                    dbgCenterRaw = centerRaw;

                    snapOffsetFromCenter(centerRaw);
                }
            }
        }

        prevMagnetPressed = pressed;
    }

    /**
     * Use the measured centerRaw to snap phaseOffsetTicks so that the virtual phase
     * aligns exactly to the nearest launch magnet phase.
     */
    private void snapOffsetFromCenter(int centerRaw) {
        if (TICKS_PER_REV <= 0) return;

        int centerPhase = wrap(centerRaw, TICKS_PER_REV);

        int l1 = wrap(MAG_LAUNCH_1_PHASE, TICKS_PER_REV);
        int l2 = wrap(MAG_LAUNCH_2_PHASE, TICKS_PER_REV);
        int l3 = wrap(MAG_LAUNCH_3_PHASE, TICKS_PER_REV);

        int dist1 = Math.abs(shortestSignedDelta(centerPhase, l1, TICKS_PER_REV));
        int dist2 = Math.abs(shortestSignedDelta(centerPhase, l2, TICKS_PER_REV));
        int dist3 = Math.abs(shortestSignedDelta(centerPhase, l3, TICKS_PER_REV));

        int expected;
        String name;
        int acceptDist;

        expected = l1;
        name = "L1";
        acceptDist = dist1;

        if (dist2 < acceptDist) {
            expected = l2;
            name = "L2";
            acceptDist = dist2;
        }
        if (dist3 < acceptDist) {
            expected = l3;
            name = "L3";
            acceptDist = dist3;
        }

        if (acceptDist > MAGNET_WINDOW_TICKS) {
            dbgLastMagnetName = "reject";
            return;
        }

        // desired offset so wrap(centerRaw + offset) == expected
        int desiredOffsetWrapped = wrap(expected - centerPhase, TICKS_PER_REV);
        int currentOffsetWrapped = wrap(phaseOffsetTicks, TICKS_PER_REV);

        int delta = shortestSignedDelta(currentOffsetWrapped, desiredOffsetWrapped, TICKS_PER_REV);
        phaseOffsetTicks += delta;

        dbgLastMagnetName = name;
        dbgLastMagnetPhase = expected;
        dbgLastSnapError = delta;
    }

    // ----------------------------
    // Motion (P-controller on virtual phase)
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

            // deadband + settle
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

            // apply minPower only when far enough away (prevents jitter near target)
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
    // Debug getters (TeleOp uses these)
    // ----------------------------

    public int getDbgRawPhase() { return dbgRawPhase; }
    public int getDbgVirtualPhase() { return dbgVirtualPhase; }
    public int getDbgPhaseOffsetTicks() { return phaseOffsetTicks; }
    public boolean getDbgMagnetPressed() { return dbgMagnetPressed; }
    public String getDbgLastMagnetName() { return dbgLastMagnetName; }
    public int getDbgLastSnapError() { return dbgLastSnapError; }
    public int getDbgError() { return dbgError; }

    // Extra optional debug if you want to print them later
    public int getDbgRiseRaw() { return dbgRiseRaw; }
    public int getDbgFallRaw() { return dbgFallRaw; }
    public int getDbgCenterRaw() { return dbgCenterRaw; }
    public int getDbgLastMagnetPhase() { return dbgLastMagnetPhase; }

    public boolean getMagnetState() { return dbgMagnetPressed; }

    public String getStatus() {
        boolean busy = isMoving();
        int mod = TICKS_PER_REV;
        if (mod <= 0) mod = 1;

        return String.format(
                "mode=%s busy=%s raw=%d rawPhase=%d vPhase=%d off=%d tgtPhase=%d err=%d lastMag=%s snap=%d manual=%s",
                indexerMotor.getMode(),
                busy,
                dbgRawEncoder,
                dbgRawPhase,
                dbgVirtualPhase,
                phaseOffsetTicks,
                wrap(targetPhase, mod),
                dbgError,
                dbgLastMagnetName,
                dbgLastSnapError,
                manualMode
        );
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

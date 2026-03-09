package org.firstinspires.ftc.teamcode.testTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.telemetry.PanelsTelemetry;

@TeleOp(name = "blue tele test")
public class Blue extends LinearOpMode {

    TelemetryManager.TelemetryWrapper panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    private enum IntakeMode { NORMAL, SORTING }


    @Configurable
    public static class Flywheel {
        public static double kP = 0.02;
        public static double kI = 0.005;
        public static double kD = 0.0;
        public static double kF = 0.00041;
        public static double targetVelocity = 0;
        public static double integralMax = 0.3;
        // Presets — set via G2 Share (close) and G2 Options (long)
        public static double closeRangeVelocity = 1250;
        public static double longRangeVelocity  = 1450;
    }

    @Configurable
    public static class Tracking {
        public static double kP = 0.01;
        public static double kI = 0.0001;
        public static double kD = 0.0008;
        public static double integralMax = 0.3;
        public static double turretPower = 0.5;
        // Turret stops moving when |tx| <= txDeadzone (target is "close enough")
        public static double txDeadzone = 0.5;
        public static boolean enableTracking = false;
    }

    @Configurable
    public static class Turntable {
        public static double manualPower = 0.5;
        public static int minTicks = -1000;
        public static int maxTicks = 282;
        public static boolean enableWraparound = true;
        public static double wraparoundKP = 0.015;
        public static double wraparoundKI = 0.0;
        public static double wraparoundKD = 0.0005;
        public static double wraparoundMaxPower = 0.8;
        public static double wraparoundTolerance = 10.0;
        public static double wraparoundIntegralMax = 0.3;
        public static double wraparoundTimeoutSec = 5.0;
    }

    @Configurable
    public static class TransferMotor {
        // Power used during the 360° shoot spin
        public static double shootPower = -1.0;
        // Power used when driver manually controls via triggers
        public static double manualPower = 1.0;
    }

    @Configurable
    public static class BallDetection {
        // SwyftRanger: distance in inches using formula (voltage * 48.7) - 4.9
        public static double detectionDistanceIn = 3.0;
        // How long to wait after start before sensors can ever activate (ms)
        public static double startupDelayMs = 2000;
    }

    @Configurable
    public static class ColorDetection {
        // Time to wait for color sensor to confirm a color before defaulting to purple
        public static double colorTimeoutMs = 2000.0;
        public static boolean enableLEDs = true;
    }

    @Configurable
    public static class ColorSensor1 {
        public static double gain = 8.0;
        // Green ranges (alpha-normalized)
        public static double greenGMin = 0.35;  public static double greenGMax = 1.0;
        public static double greenRMin = 0.0;   public static double greenRMax = 0.33;
        public static double greenBMin = 0.0;   public static double greenBMax = 0.33;
        // Purple ranges (alpha-normalized)
        public static double purpleRMin = 0.30; public static double purpleRMax = 1.0;
        public static double purpleBMin = 0.30; public static double purpleBMax = 1.0;
        public static double purpleGMin = 0.0;  public static double purpleGMax = 0.33;
    }

    @Configurable
    public static class ColorSensor2 {
        public static double gain = 8.0;
        // Green ranges (alpha-normalized)
        public static double greenGMin = 0.35;  public static double greenGMax = 1.0;
        public static double greenRMin = 0.0;   public static double greenRMax = 0.33;
        public static double greenBMin = 0.0;   public static double greenBMax = 0.33;
        // Purple ranges (alpha-normalized)
        public static double purpleRMin = 0.30; public static double purpleRMax = 1.0;
        public static double purpleBMin = 0.30; public static double purpleBMax = 1.0;
        public static double purpleGMin = 0.0;  public static double purpleGMax = 0.33;
    }

    @Configurable
    public static class SpindexerPositions {
        // Start-of-match position (both servos at 0.5)
        public static double initPosition = 0.5;

        // Intake slot positions (positive = clockwise).
        // Servo goes to slot1 when intake activates; advances one slot after each ball is loaded.
        // When the disk is at slotN, that slot is aligned with the intake/shooter opening.
        // Spinning +fullRotation from slotN fires the ball in slotN first, then slotN+1, etc.
        // Full 360° = 0.5653 servo units. Step = 0.5653 / 3 = 0.1884
        public static double slot1 = 0.097;             // ball 1 — fixed reference
        public static double slot2 = 0.097 + 0.1884;   // = 0.2854
        public static double slot3 = 0.097 + 0.3768;   // = 0.4737

        // One full 360° revolution of the spindexer in servo position units.
        public static double fullRotation = 0.5653;

        // Safety buffer: servo must have at least this much room past (startPos + fullRotation)
        // before the servo upper limit (1.0). If not, startTransfer() falls back to slot1.
        // Prevents the servo stalling against its limit mid-shoot.
        public static double shootTolerance = 0.05;

        // How long (ms) the 360° shoot spin runs — transfer motor stays at max during this window.
        // ~3300 ms gives the spindexer enough time to complete the full rotation.
        public static double transferDurationMs = 1500;

        // Delay between manual slot advances (ms)
        public static double moveWaitMs = 300;

        // Time (ms) after a servo move before sensors are enabled.
        // Prevents false detections while the spindexer is rotating.
        public static double settleTimeMs = 200;

        // Servo slew speed in servo-units per SECOND (time-based, loop-rate independent).
        // 1.0 = full range (0→1) in 1 second. One slot step = 0.2353 units.
        // Example: 0.5 = one slot in ~0.5s, 0.2 = one slot in ~1.2s, 5.0 = near-instant.
        public static double servoSpeed = 4.0;

        // Separate speed for the 360° transfer/shoot spin (servo-units per second).
        // Lower = slower spin = more time for each ball to fire through the shooter.
        public static double transferServoSpeed = 0.7;
    }

    @Configurable
    public static class TransferSettings {
        // Prevent shooting when 0 balls are loaded
        public static boolean preventZeroBallTransfer = true;
        // Safety timeout to abort a stuck transfer
        public static double transferTimeoutMs = 5000;
        public static boolean enableTransferTimeout = true;
        // Automatically enter shooting mode when 3 balls are loaded
        public static boolean autoEnterShootingAt3 = false;
    }

    @Configurable
    public static class Intake {
        public static double intakePower = 1.0;
        public static double outtakePower = -1.0;
        public static boolean autoSortEnabled = true;
        // Pause intake briefly after a ball is detected (prevents double-counting)
        public static double pauseOnDetectMs = 100;
        // Automatically stop intake after shooting
        public static boolean autoPauseAfterShoot = true;
    }

    @Configurable
    public static class Drive {
        public static double maxSpeed = 1.0;
        public static double turnSpeed = 1.0;
    }

    @Configurable
    public static class ShootingPattern {
        // Desired shooting order when in SORTING mode (comma-separated colors)
        public static String pattern = "green,purple,purple";
        // Rotate spindexer to match pattern order before shooting
        public static boolean enablePatternMatching = true;
        // Cycle the pattern after each full 3-ball shoot
        public static boolean enablePatternCycling = true;
    }

    // =========================================================
    //  HARDWARE
    // =========================================================

    private DcMotorEx motor;          // flywheel shooter
    private DcMotorEx turntable;
    private DcMotorEx intake;
    private DcMotorEx transferMotor;  // omni wheel / transfer motor
    private Servo spindexerServo1;
    private Servo spindexerServo2;
    private AnalogInput ranger;
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Limelight3A limelight;

    // =========================================================
    //  STATE
    // =========================================================

    // Flywheel PID
    private double integralSum = 0;
    private double lastError = 0;

    // Tracking PID
    private double trackingIntegralSum = 0;
    private double trackingLastError = 0;

    // Turret wraparound
    private boolean turretWraparoundActive = false;
    private int turretWraparoundTarget = 0;
    private double turretWraparoundIntegral = 0;
    private double turretWraparoundLastError = 0;
    private double turretWraparoundStartTime = 0;

    // Spindexer: -1 = initPosition, 0 = slot1, 1 = slot2, 2 = slot3
    private int spindexerStep = -1;
    private double spindexerTargetPos = 0.5;  // commanded target position
    private double spindexerCurrentPos = 0.5; // actual slewed position
    private double spindexerMoveTime = 0;     // timestamp of last servo command
    private boolean sensorsEnabled = false;   // sensors gated by spindexer settled
    private final ElapsedTime slewTimer = new ElapsedTime(); // dedicated slew timer

    // Transfer (shoot)
    private boolean transferActive = false;
    private double transferStartTime = 0;
    private int originalDetectedBalls = 0;

    // Shooter
    private boolean shooterRumbled = false;
    private boolean shootingMode = false;

    // Intake / ball tracking
    private IntakeMode intakeMode = IntakeMode.NORMAL;
    private boolean intakeToggled = false;
    private int detectedBalls = 0;
    private String[] ballSlots = new String[3];
    private boolean ballDetectedLastLoop = false;
    private boolean manualSpinUsed = false;
    private double intakePauseUntil = 0;
    private boolean intakeActive = true;

    // Color detection (SORTING mode)
    private boolean awaitingColorDetect = false;
    private double colorDetectStartTime = 0;

    // Button edge detection
    private boolean lastSquareG1 = false;
    private boolean lastSquareG2 = false;
    private boolean lastDpadLeftG1 = false;
    private boolean lastDpadDownG2 = false;
    private boolean lastDpadUpG2 = false;
    private boolean lastDpadLeftG2 = false;
    private boolean lastTriangleG1 = false;
    private boolean lastTriangleG2 = false;
    private boolean lastCircleG1 = false;
    private boolean lastCircleG2 = false;
    private boolean lastCrossG1 = false;
    private boolean lastCrossG2 = false;
    private boolean lastDpadUpG1 = false;
    private boolean lastShareG1 = false;
    private boolean lastOptionsG1 = false;
    private boolean lastShareG2 = false;
    private boolean lastOptionsG2 = false;

    // Last detected ball color (from color sensors)
    private String lastDetectedColor = null;

    // Shooting pattern
    private String[] currentPattern = new String[3];

    // Timer
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    // =========================================================
    //  MAIN
    // =========================================================

    @Override
    public void runOpMode() {

        // Flywheel
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Turntable
        turntable = hardwareMap.get(DcMotorEx.class, "turntable");
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turntable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Transfer motor (omni wheel)
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Spindexer — starts at initPosition (0.5)
        spindexerServo1 = hardwareMap.get(Servo.class, "servo1");
        spindexerServo2 = hardwareMap.get(Servo.class, "servo2");
        spindexerStep = -1;
        setSpindexerPosition(SpindexerPositions.initPosition);

        // Sensors
        ranger = hardwareMap.get(AnalogInput.class, "distanceSensor");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");

        // Drive
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
        limelight.start();

        // Init ball state
        for (int i = 0; i < 3; i++) ballSlots[i] = null;
        String[] initial = ShootingPattern.pattern.split(",");
        currentPattern = (initial.length == 3)
                ? new String[]{initial[0].trim(), initial[1].trim(), initial[2].trim()}
                : new String[]{"green", "purple", "purple"};

        panelsTelemetry.update();
        waitForStart();
        timer.reset();
        lastTime = timer.seconds();

        // ===================== MAIN LOOP =====================
        while (opModeIsActive()) {
            double currentTime = timer.seconds();
            double dt = Math.max(currentTime - lastTime, 1e-3);
            lastTime = currentTime;

            // --- Drive ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * Drive.turnSpeed;
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeft.setPower((y + x + rx) / denom * Drive.maxSpeed);
            backLeft.setPower((y - x + rx) / denom * Drive.maxSpeed);
            frontRight.setPower((y - x - rx) / denom * Drive.maxSpeed);
            backRight.setPower((y + x - rx) / denom * Drive.maxSpeed);

            updateSpindexerSlew(dt);
            handleFlywheel(dt);
            handleTurret(dt);
            handleTransferMotor();
            handleShootingControls();
            handleTransfer();
            handleIntakeModeToggle();

            double nowMs = timer.milliseconds();

            // --- Sensor gating: startup delay + spindexer physically settled ---
            if (!sensorsEnabled
                    && timer.milliseconds() >= BallDetection.startupDelayMs
                    && spindexerSettled()) {
                sensorsEnabled = true;
            }
            // Color sensor LEDs on when sensors active and in SORTING mode
            setColorLEDs(sensorsEnabled && intakeMode == IntakeMode.SORTING && ColorDetection.enableLEDs);

            // --- Distance sensor ball detection ---
            if (nowMs > intakePauseUntil) intakeActive = true;

            boolean ballNow = sensorsEnabled && detectBall();
            boolean ballAdded = false;

            if (sensorsEnabled && intakeToggled && !shootingMode && !transferActive && !manualSpinUsed && intakeActive) {
                if (ballNow && !ballDetectedLastLoop && detectedBalls < 3) {
                    // Try to get color; fall back to "unknown" if sensors not ready
                    String color = detectBallColor();
                    lastDetectedColor = (color != null) ? color : "unknown";
                    commitBall(lastDetectedColor);
                    ballAdded = true;
                }
            }

            // Update color telemetry even when not detecting a ball
            if (sensorsEnabled && intakeMode == IntakeMode.SORTING) {
                String c = detectBallColor();
                if (c != null) lastDetectedColor = c;
            }

            if (ballAdded) {
                intakePauseUntil = nowMs + Intake.pauseOnDetectMs;
                intakeActive = false;
            }
            if (manualSpinUsed) {
                manualSpinUsed = false;
                ballDetectedLastLoop = false;
            }
            ballDetectedLastLoop = ballNow;

            // --- Intake toggle (cross) ---
            boolean g1Cross = gamepad1.cross;
            if (g1Cross && !lastCrossG1) toggleIntake();
            lastCrossG1 = g1Cross;

            boolean g2Cross = gamepad2.cross;
            if (g2Cross && !lastCrossG2) toggleIntake();
            lastCrossG2 = g2Cross;

            // --- Outtake (G1 dpad_right held) ---
            if (gamepad1.dpad_right) {
                intake.setPower(Intake.outtakePower);
            } else if (intakeToggled && intakeActive) {
                intake.setPower(Intake.intakePower);
            } else {
                intake.setPower(0);
            }

            LLResult result = limelight.getLatestResult();
            updateTelemetry(result);
        }

        // --- Cleanup ---
        motor.setPower(0);
        turntable.setPower(0);
        intake.setPower(0);
        transferMotor.setPower(0);
        setSpindexerPosition(SpindexerPositions.initPosition);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        // setColorLEDs(false);
        limelight.stop();
    }

    // =========================================================
    //  INTAKE HELPERS
    // =========================================================

    private void toggleIntake() {
        intakeToggled = !intakeToggled;
        // On first activation, move spindexer from init (0.5) to slot1 (0.097)
        if (intakeToggled && spindexerStep == -1) {
            spindexerStep = 0;
            setSpindexerPosition(SpindexerPositions.slot1);
        }
    }

    /** Add a ball to the slot array, advance the spindexer, and handle full-load logic. */
    private void commitBall(String color) {
        addBall(color);
        detectedBalls++;
        advanceSpindexer();
        awaitingColorDetect = false;
        if (detectedBalls == 3) {
            if (TransferSettings.autoEnterShootingAt3) {
                shootingMode = true;
                rotateForShooting();
                gamepad1.rumble(1.0, 1.0, 500);
            } else {
                // Rumble to signal full load — go to shooting zone
                gamepad1.rumble(0.5, 0.5, 300);
                gamepad2.rumble(0.5, 0.5, 300);
            }
        }
    }

    // =========================================================
    //  INTAKE MODE TOGGLE — G2 dpad_left
    // =========================================================

    private void handleIntakeModeToggle() {
        boolean btn = gamepad2.dpad_left;
        if (btn && !lastDpadLeftG2) {
            intakeMode = (intakeMode == IntakeMode.NORMAL) ? IntakeMode.SORTING : IntakeMode.NORMAL;
            awaitingColorDetect = false;
            boolean sorting = intakeMode == IntakeMode.SORTING;
            gamepad2.rumble(sorting ? 0.8 : 0.3, sorting ? 0.8 : 0.3, 200);
        }
        lastDpadLeftG2 = btn;
    }

    // =========================================================
    //  COLOR DETECTION
    // =========================================================

    /** Returns "green", "purple", or null. Each sensor uses its own ranges — either passing = detected. */
    private String detectBallColor() {
        colorSensor1.setGain((float) ColorSensor1.gain);
        colorSensor2.setGain((float) ColorSensor2.gain);
        NormalizedRGBA c1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA c2 = colorSensor2.getNormalizedColors();
        float a1 = Math.max(c1.alpha, 1e-6f);
        float a2 = Math.max(c2.alpha, 1e-6f);
        double r1 = c1.red/a1, g1 = c1.green/a1, b1 = c1.blue/a1;
        double r2 = c2.red/a2, g2 = c2.green/a2, b2 = c2.blue/a2;
        if (cs1IsGreen(r1,g1,b1) || cs2IsGreen(r2,g2,b2))   return "green";
        if (cs1IsPurple(r1,g1,b1) || cs2IsPurple(r2,g2,b2)) return "purple";
        return null;
    }

    private static boolean cs1IsGreen(double r, double g, double b) {
        return g >= ColorSensor1.greenGMin && g <= ColorSensor1.greenGMax
            && r >= ColorSensor1.greenRMin && r <= ColorSensor1.greenRMax
            && b >= ColorSensor1.greenBMin && b <= ColorSensor1.greenBMax;
    }
    private static boolean cs1IsPurple(double r, double g, double b) {
        return r >= ColorSensor1.purpleRMin && r <= ColorSensor1.purpleRMax
            && b >= ColorSensor1.purpleBMin && b <= ColorSensor1.purpleBMax
            && g >= ColorSensor1.purpleGMin && g <= ColorSensor1.purpleGMax;
    }
    private static boolean cs2IsGreen(double r, double g, double b) {
        return g >= ColorSensor2.greenGMin && g <= ColorSensor2.greenGMax
            && r >= ColorSensor2.greenRMin && r <= ColorSensor2.greenRMax
            && b >= ColorSensor2.greenBMin && b <= ColorSensor2.greenBMax;
    }
    private static boolean cs2IsPurple(double r, double g, double b) {
        return r >= ColorSensor2.purpleRMin && r <= ColorSensor2.purpleRMax
            && b >= ColorSensor2.purpleBMin && b <= ColorSensor2.purpleBMax
            && g >= ColorSensor2.purpleGMin && g <= ColorSensor2.purpleGMax;
    }

    private void setColorLEDs(boolean on) {
        if (colorSensor1 instanceof SwitchableLight) ((SwitchableLight) colorSensor1).enableLight(on);
        if (colorSensor2 instanceof SwitchableLight) ((SwitchableLight) colorSensor2).enableLight(on);
    }

    // =========================================================
    //  SPINDEXER SERVO
    // =========================================================

    /** Sets the target position. Actual movement is slewed in updateSpindexerSlew(). */
    private void setSpindexerPosition(double pos) {
        spindexerTargetPos = Math.max(0.0, Math.min(1.0, pos));
        spindexerMoveTime = timer.milliseconds();
        sensorsEnabled = false;
    }

    /** Call every loop — slews spindexer toward target using its own timer (loop-rate independent). */
    private void updateSpindexerSlew(double dt) {
        double elapsed = slewTimer.seconds();
        slewTimer.reset();

        double speed = transferActive
                ? SpindexerPositions.transferServoSpeed
                : SpindexerPositions.servoSpeed;
        double maxStep = speed * elapsed;

        double error = spindexerTargetPos - spindexerCurrentPos;
        if (Math.abs(error) <= maxStep) {
            spindexerCurrentPos = spindexerTargetPos;
        } else {
            spindexerCurrentPos += Math.signum(error) * maxStep;
        }
        spindexerServo1.setPosition(spindexerCurrentPos);
        spindexerServo2.setPosition(1.0 - spindexerCurrentPos);
    }

    /** True when spindexer has settled: time elapsed AND servo has physically reached target. */
    private boolean spindexerSettled() {
        boolean timeOk = (timer.milliseconds() - spindexerMoveTime) >= SpindexerPositions.settleTimeMs;
        boolean posOk  = Math.abs(spindexerCurrentPos - spindexerTargetPos) <= 0.02;
        return timeOk && posOk;
    }

    /** Returns the servo position for a given intake slot (0-indexed). */
    private double getSlotPosition(int slot) {
        switch (slot) {
            case 0:  return SpindexerPositions.slot1;
            case 1:  return SpindexerPositions.slot2;
            case 2:  return SpindexerPositions.slot3;
            default: return SpindexerPositions.initPosition;
        }
    }

    /** Advance to the next empty intake slot (skips filled slots). Capped at slot3. */
    private void advanceSpindexer() {
        int start = spindexerStep;
        for (int i = 1; i <= 2; i++) {
            int candidate = start + i;
            if (candidate > 2) break;
            if (ballSlots[candidate] == null) {
                spindexerStep = candidate;
                setSpindexerPosition(getSlotPosition(spindexerStep));
                return;
            }
        }
        // No empty slot found — just advance one (capped at slot3)
        if (spindexerStep < 2) {
            spindexerStep++;
            setSpindexerPosition(getSlotPosition(spindexerStep));
        }
    }

    /** Advance one slot with wrapping — used only during shoot-order alignment. */
    private void advanceSpindexerWrapping() {
        spindexerStep = (spindexerStep + 1) % 3;
        setSpindexerPosition(getSlotPosition(spindexerStep));
    }

    /** Reverse one slot (back toward slot1, or to initPosition if already at slot1). */
    private void reverseSpindexer() {
        if (spindexerStep > 0) {
            spindexerStep--;
            setSpindexerPosition(getSlotPosition(spindexerStep));
        } else {
            spindexerStep = -1;
            setSpindexerPosition(SpindexerPositions.initPosition);
        }
    }

    // =========================================================
    //  BALL SLOT TRACKING
    // =========================================================

    private void addBall(String color) {
        // Shift left, newest ball at index 2
        ballSlots[0] = ballSlots[1];
        ballSlots[1] = ballSlots[2];
        ballSlots[2] = color;
    }

    /** Reset all ball state and return spindexer to slot1 for next intake cycle. */
    private void clearBalls() {
        for (int i = 0; i < 3; i++) ballSlots[i] = null;
        detectedBalls = 0;
        ballDetectedLastLoop = false;
        awaitingColorDetect = false;
        spindexerStep = 0;
        // Return to slot1 (0.097) — ready for next intake
        setSpindexerPosition(SpindexerPositions.slot1);
    }

    /**
     * Positions the spindexer so the 360° shoot spin fires balls in the right order.
     *
     * Physical rule (clockwise = positive = increasing servo position):
     *   When disk is at slotN position, the ball IN slotN fires FIRST during the spin.
     *   Intake mode start (slot1 / 0.097) is always the first-to-shoot reference position.
     *
     * NORMAL mode: reset disk to slot1 — no color sorting, just spin from there.
     * SORTING mode: find which slot holds the desired first ball, move disk there,
     *               then reorder the ballSlots array to reflect the actual shoot sequence.
     */
    private void rotateForShooting() {
        if (intakeMode != IntakeMode.SORTING || !ShootingPattern.enablePatternMatching || detectedBalls == 0) {
            // Normal mode: always start from slot1 (intake mode start = first to shoot).
            spindexerStep = 0;
            setSpindexerPosition(SpindexerPositions.slot1);
            return;
        }

        // SORTING mode -------------------------------------------------------
        // ballSlots[i] = ball that was loaded at getSlotPosition(i).
        // When disk is at getSlotPosition(i), that ball is at the shooter opening → fires first.
        // Find which slot index holds the ball matching currentPattern[0].
        String wantFirst = currentPattern[0].trim();
        int firstIdx = -1;

        // Exact color match
        for (int i = 0; i < detectedBalls; i++) {
            if (wantFirst.equals(ballSlots[i])) { firstIdx = i; break; }
        }
        // Fallback: first slot that is not null (use whatever is available)
        if (firstIdx == -1) {
            for (int i = 0; i < detectedBalls; i++) {
                if (ballSlots[i] != null) { firstIdx = i; break; }
            }
        }
        if (firstIdx == -1) firstIdx = 0;

        // Move disk so the desired first ball is at the shooter opening.
        spindexerStep = firstIdx;
        setSpindexerPosition(getSlotPosition(firstIdx));

        // Reorder ballSlots so index 0 = first to shoot, wrapping through the 3 slots.
        String a = ballSlots[firstIdx % 3];
        String b = ballSlots[(firstIdx + 1) % 3];
        String c = ballSlots[(firstIdx + 2) % 3];
        ballSlots[0] = a;
        ballSlots[1] = b;
        ballSlots[2] = c;

        // Feedback rumbles
        if (firstIdx != 0) {
            gamepad1.rumble(0.5, 0.5, 200); // position adjusted
        }
        if (!wantFirst.equals(ballSlots[0]) && !"unknown".equals(ballSlots[0])) {
            gamepad1.rumble(0.8, 0.8, 400); // pattern mismatch — no exact match found
        }
    }

    private String formatBallSlots() {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < 3; i++) {
            sb.append(ballSlots[i] == null ? "empty" : ballSlots[i].equals("unknown") ? "???" : ballSlots[i]);
            if (i < 2) sb.append(", ");
        }
        return sb.append("]").toString();
    }

    private String formatShootingOrder() {
        if (detectedBalls == 0) return "No balls";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < detectedBalls; i++) {
            sb.append(ballSlots[i] == null ? "?" : ballSlots[i].equals("unknown") ? "???" : ballSlots[i]);
            if (i < detectedBalls - 1) sb.append(" > ");
        }
        return sb.toString();
    }

    // =========================================================
    //  TRANSFER (360° SHOOT SPIN)
    // =========================================================

    private void startTransfer() {
        // rotateForShooting() has already positioned the disk at the correct slot.
        // Read back the current servo start position.
        double startPos = (spindexerStep >= 0) ? getSlotPosition(spindexerStep) : SpindexerPositions.slot1;

        // Safety check: ensure startPos + fullRotation fits within servo range [0, 1].
        // If not (e.g. disk is too close to servo max), fall back to slot1.
        // shootTolerance is a configurable buffer so the servo never stalls at the limit.
        double maxAllowedStart = 1.0 - SpindexerPositions.fullRotation - SpindexerPositions.shootTolerance;
        if (startPos > maxAllowedStart) {
            startPos = SpindexerPositions.slot1;
            spindexerStep = 0;
            gamepad1.rumble(0.3, 0.3, 100); // subtle feedback: reset to slot1
        }

        // Command servo to spin one full 360° clockwise (increasing servo position).
        // The disk passes each loaded slot through the shooter opening, firing all balls.
        setSpindexerPosition(startPos + SpindexerPositions.fullRotation);

        originalDetectedBalls = detectedBalls;
        transferActive = true;
        transferStartTime = timer.milliseconds();
    }

    private void handleTransfer() {
        if (!transferActive) return;

        double elapsed = timer.milliseconds() - transferStartTime;

        // Safety timeout
        if (TransferSettings.enableTransferTimeout && elapsed > TransferSettings.transferTimeoutMs) {
            transferActive = false;
            shootingMode = false;
            clearBalls();
            gamepad1.rumble(1.0, 1.0, 300);
            return;
        }

        // Transfer complete when duration elapses
        if (elapsed >= SpindexerPositions.transferDurationMs) {
            transferActive = false;
            shootingMode = false;

            if (ShootingPattern.enablePatternCycling && originalDetectedBalls == 3) {
                // Cycle pattern for next round (rotate left)
                String t = currentPattern[0];
                currentPattern[0] = currentPattern[1];
                currentPattern[1] = currentPattern[2];
                currentPattern[2] = t;
                gamepad1.rumble(0.4, 0.4, 150);
            }

            clearBalls();

            if (Intake.autoPauseAfterShoot) {
                intakeToggled = false;
                gamepad1.rumble(0.3, 0.3, 200);
            }
        }
        // Transfer motor power is handled in handleTransferMotor()
    }

    // =========================================================
    //  TRANSFER MOTOR (OMNI WHEEL)
    // =========================================================

    private void handleTransferMotor() {
        if (transferActive) {
            // Full power during the 360° shoot
            transferMotor.setPower(TransferMotor.shootPower);
            return;
        }
        // Manual control via triggers
        double power = 0;
        if (gamepad1.right_trigger > 0.1) power += gamepad1.right_trigger;
        if (gamepad1.left_trigger > 0.1)  power -= gamepad1.left_trigger;
        if (gamepad2.right_trigger > 0.1) power += gamepad2.right_trigger;
        if (gamepad2.left_trigger > 0.1)  power -= gamepad2.left_trigger;
        transferMotor.setPower(Math.max(-1.0, Math.min(1.0, power)) * TransferMotor.manualPower);
    }

    // =========================================================
    //  FLYWHEEL PID
    // =========================================================

    private void handleFlywheel(double dt) {
        double velocity = motor.getVelocity();
        double error = Flywheel.targetVelocity - velocity;

        integralSum = Math.max(-Flywheel.integralMax,
                Math.min(Flywheel.integralMax, integralSum + error * dt));
        double output = (Flywheel.kP * error)
                + (Flywheel.kI * integralSum)
                + (Flywheel.kD * (error - lastError) / dt)
                + (Flywheel.kF * Flywheel.targetVelocity);
        output = Math.max(-1.0, Math.min(1.0, output));
        lastError = error;

        if (Flywheel.targetVelocity == 0) {
            integralSum = 0;
            lastError = 0;
            motor.setPower(0);
            shooterRumbled = false;
        } else {
            motor.setPower(output);
            if (Math.abs(error) <= 50 && !shooterRumbled) {
                gamepad1.rumble(1.0, 1.0, 1000);
                shooterRumbled = true;
            }
        }
    }

    // =========================================================
    //  TURRET — TRACKING + WRAPAROUND
    // =========================================================

    private void handleTurret(double dt) {
        // Toggle limelight tracking — G1 Dpad Up (or G2 Circle)
        boolean g1DpadUp = gamepad1.dpad_up;
        if (g1DpadUp && !lastDpadUpG1) {
            Tracking.enableTracking = !Tracking.enableTracking;
            if (Tracking.enableTracking) {
                trackingIntegralSum = 0;
                trackingLastError = 0;
                gamepad1.rumble(0.8, 0.8, 300);   // ON — strong double rumble
            } else {
                turntable.setPower(0);
                gamepad1.rumble(0.3, 0.3, 150);   // OFF — soft rumble
            }
        }
        lastDpadUpG1 = g1DpadUp;

        boolean g2Circle = gamepad2.circle;
        if (g2Circle && !lastCircleG2) {
            Tracking.enableTracking = !Tracking.enableTracking;
            if (Tracking.enableTracking) {
                trackingIntegralSum = 0;
                trackingLastError = 0;
                gamepad2.rumble(0.8, 0.8, 300);
            } else {
                turntable.setPower(0);
                gamepad2.rumble(0.3, 0.3, 150);
            }
        }
        lastCircleG2 = g2Circle;

        // Wraparound
        if (Turntable.enableWraparound) {
            int pos = turntable.getCurrentPosition();
            if (!turretWraparoundActive) {
                if (pos > Turntable.maxTicks) {
                    startWraparound(Turntable.minTicks);
                } else if (pos < Turntable.minTicks) {
                    startWraparound(Turntable.maxTicks);
                }
            }
            if (turretWraparoundActive) {
                double elapsed = timer.seconds() - turretWraparoundStartTime;
                boolean cancel = (gamepad1.left_bumper && gamepad1.right_bumper)
                        || (gamepad2.left_bumper && gamepad2.right_bumper);
                if (cancel || elapsed > Turntable.wraparoundTimeoutSec) {
                    cancelWraparound(elapsed > Turntable.wraparoundTimeoutSec);
                } else {
                    double wErr = turretWraparoundTarget - pos;
                    if (Math.abs(wErr) <= Turntable.wraparoundTolerance) {
                        finishWraparound();
                    } else {
                        turretWraparoundIntegral = Math.max(-Turntable.wraparoundIntegralMax,
                                Math.min(Turntable.wraparoundIntegralMax,
                                        turretWraparoundIntegral + wErr * dt));
                        double wOut = (Turntable.wraparoundKP * wErr)
                                + (Turntable.wraparoundKI * turretWraparoundIntegral)
                                + (Turntable.wraparoundKD * (wErr - turretWraparoundLastError) / dt);
                        turntable.setPower(Math.max(-Turntable.wraparoundMaxPower,
                                Math.min(Turntable.wraparoundMaxPower, wOut)));
                        turretWraparoundLastError = wErr;
                    }
                }
            }
        }

        if (!turretWraparoundActive) {
            if (Tracking.enableTracking) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx();
                    if (Math.abs(tx) > Tracking.txDeadzone) {
                        double tErr = -tx;
                        // Reset integral on direction change to prevent overshoot
                        if (trackingLastError != 0 && Math.signum(tErr) != Math.signum(trackingLastError))
                            trackingIntegralSum = 0;
                        trackingIntegralSum = Math.max(-Tracking.integralMax,
                                Math.min(Tracking.integralMax, trackingIntegralSum + tErr * dt));
                        double tOut = (Tracking.kP * tErr)
                                + (Tracking.kI * trackingIntegralSum)
                                + (Tracking.kD * (tErr - trackingLastError) / dt);
                        turntable.setPower(-Math.max(-Tracking.turretPower,
                                Math.min(Tracking.turretPower, tOut)));
                        trackingLastError = tErr;
                    } else {
                        // Within deadzone — locked on target
                        turntable.setPower(0);
                        trackingIntegralSum = 0;
                        trackingLastError = 0;
                    }
                } else {
                    turntable.setPower(0);
                    trackingIntegralSum = 0;
                    trackingLastError = 0;
                }
            } else {
                if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    turntable.setPower(Turntable.manualPower);
                } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    turntable.setPower(-Turntable.manualPower);
                } else {
                    turntable.setPower(0);
                }
                trackingIntegralSum = 0;
                trackingLastError = 0;
            }
        } else {
            trackingIntegralSum = 0;
            trackingLastError = 0;
        }
    }

    private void startWraparound(int target) {
        turretWraparoundActive = true;
        turretWraparoundTarget = target;
        turretWraparoundIntegral = 0;
        turretWraparoundLastError = 0;
        turretWraparoundStartTime = timer.seconds();
        gamepad1.rumble(0.3, 0.3, 150);
    }

    private void cancelWraparound(boolean timedOut) {
        turretWraparoundActive = false;
        turretWraparoundIntegral = 0;
        turretWraparoundLastError = 0;
        turntable.setPower(0);
        gamepad1.rumble(1.0, 1.0, 300);
        gamepad2.rumble(1.0, 1.0, 300);
        if (timedOut) Turntable.enableWraparound = false;
    }

    private void finishWraparound() {
        turretWraparoundActive = false;
        turretWraparoundIntegral = 0;
        turretWraparoundLastError = 0;
        turntable.setPower(0);
        gamepad1.rumble(0.5, 0.5, 200);
    }

    // =========================================================
    //  SHOOTING CONTROLS
    // =========================================================

    private void handleShootingControls() {
        // Triangle (G1 or G2) = enter shooting mode
        boolean g1Tri = gamepad1.triangle;
        if (g1Tri && !lastTriangleG1 && !shootingMode && !transferActive) {
            shootingMode = true;
            rotateForShooting();
        }
        lastTriangleG1 = g1Tri;

        boolean g2Tri = gamepad2.triangle;
        if (g2Tri && !lastTriangleG2 && !shootingMode && !transferActive) {
            shootingMode = true;
            rotateForShooting();
        }
        lastTriangleG2 = g2Tri;

        // Circle (G1) = start 360° transfer / shoot
        boolean g1Circle = gamepad1.circle;
        if (g1Circle && !lastCircleG1 && shootingMode && !transferActive) {
            if (TransferSettings.preventZeroBallTransfer && detectedBalls == 0) {
                gamepad1.rumble(0.5, 0.5, 200);
            } else {
                startTransfer();
            }
        }
        lastCircleG1 = g1Circle;

        // G2 dpad_down = reverse one slot (eject last ball)
        boolean g2DpadDown = gamepad2.dpad_down;
        if (g2DpadDown && !lastDpadDownG2 && !transferActive) {
            reverseSpindexer();
            manualSpinUsed = true;
            if (detectedBalls > 0) {
                ballSlots[0] = ballSlots[1];
                ballSlots[1] = ballSlots[2];
                ballSlots[2] = null;
                detectedBalls--;
                if (shootingMode && detectedBalls == 0) shootingMode = false;
            }
            gamepad2.rumble(0.5, 0.5, 300);
        }
        lastDpadDownG2 = g2DpadDown;

        // G2 dpad_up = toggle pattern matching on/off
        boolean g2DpadUp = gamepad2.dpad_up;
        if (g2DpadUp && !lastDpadUpG2) {
            ShootingPattern.enablePatternMatching = !ShootingPattern.enablePatternMatching;
            gamepad2.rumble(ShootingPattern.enablePatternMatching ? 0.5 : 0.3,
                    ShootingPattern.enablePatternMatching ? 0.5 : 0.3, 200);
        }
        lastDpadUpG2 = g2DpadUp;

        // G2 Square = manually add unknown ball + advance spindexer
        boolean g2Sq = gamepad2.square;
        if (g2Sq && !lastSquareG2 && !shootingMode && !transferActive) {
            if (detectedBalls < 3) {
                addBall("unknown");
                detectedBalls++;
                advanceSpindexer();
                if (detectedBalls == 3) gamepad2.rumble(0.5, 0.5, 200);
            } else {
                advanceSpindexer();
            }
            manualSpinUsed = true;
        }
        lastSquareG2 = g2Sq;

        // G1 Square = manually add unknown ball + advance spindexer
        boolean g1Sq = gamepad1.square;
        if (g1Sq && !lastSquareG1 && !shootingMode && !transferActive) {
            if (detectedBalls < 3) {
                addBall("unknown");
                detectedBalls++;
                advanceSpindexer();
                if (detectedBalls == 3) gamepad1.rumble(0.5, 0.5, 200);
            } else {
                advanceSpindexer();
            }
            manualSpinUsed = true;
        }
        lastSquareG1 = g1Sq;

        // G1 dpad_left = reverse one slot to unjam (does NOT change ball count)
        // Use this if the spindexer jams mid-rotation, then manually re-register with Square
        boolean g1DpadLeft = gamepad1.dpad_left;
        if (g1DpadLeft && !lastDpadLeftG1 && !transferActive) {
            reverseSpindexer();
            manualSpinUsed = true;
            gamepad1.rumble(0.5, 0.5, 100);
        }
        lastDpadLeftG1 = g1DpadLeft;

        // G1 Share = close range flywheel preset
        boolean g1Share = gamepad1.share;
        if (g1Share && !lastShareG1) {
            Flywheel.targetVelocity = Flywheel.closeRangeVelocity;
            shooterRumbled = false;
            gamepad1.rumble(0.3, 0.0, 150);
        }
        lastShareG1 = g1Share;

        // G1 Options = long range flywheel preset
        boolean g1Options = gamepad1.options;
        if (g1Options && !lastOptionsG1) {
            Flywheel.targetVelocity = Flywheel.longRangeVelocity;
            shooterRumbled = false;
            gamepad1.rumble(0.0, 0.3, 150);
        }
        lastOptionsG1 = g1Options;

        // G2 Share = close range flywheel preset
        boolean g2Share = gamepad2.share;
        if (g2Share && !lastShareG2) {
            Flywheel.targetVelocity = Flywheel.closeRangeVelocity;
            shooterRumbled = false;
            gamepad2.rumble(0.3, 0.0, 150);
        }
        lastShareG2 = g2Share;

        // G2 Options = long range flywheel preset
        boolean g2Options = gamepad2.options;
        if (g2Options && !lastOptionsG2) {
            Flywheel.targetVelocity = Flywheel.longRangeVelocity;
            shooterRumbled = false;
            gamepad2.rumble(0.0, 0.3, 150);
        }
        lastOptionsG2 = g2Options;

        // G2 Cross = stop flywheel
        // (cross already used for intake toggle — skip if already mapped)
    }

    // =========================================================
    //  BALL DETECTION
    // =========================================================

    private boolean detectBall() {
        double inches = (ranger.getVoltage() * 48.7) - 4.9;
        return inches <= BallDetection.detectionDistanceIn;
    }

    // =========================================================
    //  TELEMETRY
    // =========================================================

    private void updateTelemetry(LLResult result) {
        boolean limelightValid = (result != null && result.isValid());

        // --- Driver station ---
        telemetry.addData("Intake", intakeToggled ? "ON" : "OFF");
        telemetry.addData("Mode", intakeMode.name() + (intakeMode == IntakeMode.SORTING ? " (color)" : " (distance only)"));
        telemetry.addData("Balls", detectedBalls + "/3  " + formatBallSlots());
        telemetry.addData("Shoot Mode", shootingMode ? "READY -> " + formatShootingOrder() : "COLLECT");
        // if (awaitingColorDetect) {
        //     double remaining = colorDetectStartTime + ColorDetection.colorTimeoutMs - timer.milliseconds();
        //     telemetry.addData("Color Detect", (int) remaining + "ms left");
        // }
        telemetry.addData("Transfer", transferActive ? "SHOOTING" : "IDLE");
        telemetry.addData("Sensors", sensorsEnabled ? "ON" : "WAITING (settling)");
        double distIn = (ranger.getVoltage() * 48.7) - 4.9;
        telemetry.addData("Distance (in)", String.format("%.2f  [thresh=%.2f]  %s", distIn, BallDetection.detectionDistanceIn, distIn <= BallDetection.detectionDistanceIn ? "BALL" : "none"));
        telemetry.addData("Spindexer",
                (spindexerStep == -1 ? "INIT" : "Slot " + (spindexerStep + 1))
                + "  pos=" + String.format("%.3f", currentServoPos()));
        telemetry.addData("Pattern Match", ShootingPattern.enablePatternMatching ? "ON (G2 dpad_up)" : "OFF (G2 dpad_up)");
        telemetry.addData("Tracking", Tracking.enableTracking ? "ON  [G1 dpad_up]" : "OFF [G1 dpad_up]");
        if (Tracking.enableTracking && limelightValid) {
            double tx = limelight.getLatestResult().getTx();
            telemetry.addData("tx", String.format("%.2f  dead=%.2f  %s",
                    tx, Tracking.txDeadzone, Math.abs(tx) <= Tracking.txDeadzone ? "LOCKED" : "tracking"));
        }
        telemetry.addData("Turret", turretWraparoundActive
                ? "WRAPPING -> " + turretWraparoundTarget
                : turntable.getCurrentPosition() + " ticks");
        // telemetry.addData("Distance (in)", String.format("%.2f", (ranger.getVoltage() * 48.7) - 4.9));
        // telemetry.addData("Ranger Voltage", String.format("%.3f", ranger.getVoltage()));
        telemetry.addData("Flywheel", String.format("actual=%.0f  target=%.0f  err=%.0f",
                motor.getVelocity(), Flywheel.targetVelocity, Flywheel.targetVelocity - motor.getVelocity()));
        telemetry.addData("Color Detected", lastDetectedColor != null ? lastDetectedColor.toUpperCase() : "NONE");
        telemetry.addData("Limelight", limelightValid ? "LOCKED" : "NO TARGET");
        telemetry.update();

        // --- Panels dashboard ---
        panelsTelemetry.addData("=== INTAKE ===", "");
        panelsTelemetry.addData("Intake", intakeToggled ? "ON" : "OFF");
        panelsTelemetry.addData("Mode", intakeMode.name() + "  [G2 dpad_left to toggle]");
        panelsTelemetry.addData("Balls Loaded", detectedBalls + "/3");
        panelsTelemetry.addData("Ball Slots", formatBallSlots());
        panelsTelemetry.addData("Shoot Order", formatShootingOrder());
        // if (awaitingColorDetect) {
        //     double ms = colorDetectStartTime + ColorDetection.colorTimeoutMs - timer.milliseconds();
        //     panelsTelemetry.addData("Color Timer", (int) ms + "ms remaining");
        // }
        panelsTelemetry.addData("", "");

        panelsTelemetry.addData("=== SPINDEXER ===", "");
        double sp = currentServoPos();
        panelsTelemetry.addData("Slot", spindexerStep == -1 ? "INIT (0.5)" : "Slot " + (spindexerStep + 1));
        panelsTelemetry.addData("Servo1 pos", String.format("%.3f", sp));
        panelsTelemetry.addData("Servo2 pos", String.format("%.3f", 1.0 - sp));
        panelsTelemetry.addData("Slot1", SpindexerPositions.slot1);
        panelsTelemetry.addData("Slot2", SpindexerPositions.slot2);
        panelsTelemetry.addData("Slot3", SpindexerPositions.slot3);
        panelsTelemetry.addData("Full Rotation", SpindexerPositions.fullRotation);
        panelsTelemetry.addData("Transfer", transferActive
                ? "ACTIVE  " + (int) Math.max(0, SpindexerPositions.transferDurationMs - (timer.milliseconds() - transferStartTime)) + "ms left"
                : "IDLE");
        panelsTelemetry.addData("", "");

        panelsTelemetry.addData("=== DISTANCE SENSOR ===", "");
        double panelDistIn = (ranger.getVoltage() * 48.7) - 4.9;
        panelsTelemetry.addData("Ranger Voltage", String.format("%.4fV", ranger.getVoltage()));
        panelsTelemetry.addData("Distance (in)", String.format("%.3f", panelDistIn));
        panelsTelemetry.addData("Threshold (in)", BallDetection.detectionDistanceIn);
        panelsTelemetry.addData("Ball Detected", panelDistIn <= BallDetection.detectionDistanceIn ? "YES" : "NO");
        panelsTelemetry.addData("Sensors", sensorsEnabled ? "ON" : "WAITING (spindexer settling)");
        panelsTelemetry.addData("", "");

        // Color sensors — still disabled
        // if (intakeMode == IntakeMode.SORTING) {
        //     NormalizedRGBA c1 = colorSensor1.getNormalizedColors();
        //     NormalizedRGBA c2 = colorSensor2.getNormalizedColors();
        //     float avgA = (c1.alpha + c2.alpha) / 2f;
        //     double nR = (avgA > 0) ? (c1.red + c2.red) / 2f / avgA : 0;
        //     double nG = (avgA > 0) ? (c1.green + c2.green) / 2f / avgA : 0;
        //     double nB = (avgA > 0) ? (c1.blue + c2.blue) / 2f / avgA : 0;
        //     panelsTelemetry.addData("Norm R/G/B", String.format("%.3f / %.3f / %.3f", nR, nG, nB));
        //     panelsTelemetry.addData("Alpha (avg)", String.format("%.3f", (double) avgA));
        //     panelsTelemetry.addData("Gain", ColorDetection.gain);
        // }
        panelsTelemetry.addData("", "");

        panelsTelemetry.addData("=== SHOOTER ===", "");
        panelsTelemetry.addData("Pattern", String.join(" > ", currentPattern));
        panelsTelemetry.addData("Pattern Match", ShootingPattern.enablePatternMatching ? "ON" : "OFF");
        panelsTelemetry.addData("Flywheel Target (tks/s)", String.format("%.1f", Flywheel.targetVelocity));
        panelsTelemetry.addData("Flywheel Actual (tks/s)", String.format("%.1f", motor.getVelocity()));
        panelsTelemetry.addData("Flywheel Error", String.format("%.1f", Flywheel.targetVelocity - motor.getVelocity()));
        panelsTelemetry.addData("Flywheel Power", String.format("%.3f", motor.getPower()));
        panelsTelemetry.addData("Close Range Preset", Flywheel.closeRangeVelocity + "  [G2 Share]");
        panelsTelemetry.addData("Long Range Preset", Flywheel.longRangeVelocity + "  [G2 Options]");
        panelsTelemetry.addData("Transfer Motor", String.format("%.3f", transferMotor.getPower()));
        panelsTelemetry.addData("", "");
        panelsTelemetry.addData("=== COLOR DETECTION ===", "");
        panelsTelemetry.addData("Last Detected", lastDetectedColor != null ? lastDetectedColor.toUpperCase() : "NONE");
        panelsTelemetry.addData("Sensors", sensorsEnabled ? "ON" : "WAITING (spindexer moving)");
        panelsTelemetry.addData("", "");

        panelsTelemetry.addData("=== TURRET ===", "");
        if (turretWraparoundActive) {
            panelsTelemetry.addData("Turret", "WRAPPING -> " + turretWraparoundTarget);
            panelsTelemetry.addData("Wrap Error", turretWraparoundTarget - turntable.getCurrentPosition());
        } else {
            panelsTelemetry.addData("Turret Ticks", turntable.getCurrentPosition());
            panelsTelemetry.addData("Turret Power", String.format("%.3f", turntable.getPower()));
        }
        panelsTelemetry.addData("Tracking", Tracking.enableTracking ? "ON  [G1 dpad_up]" : "OFF [G1 dpad_up]");
        panelsTelemetry.addData("tx Deadzone", String.format("+/- %.2f", Tracking.txDeadzone));
        if (limelightValid) {
            double tx = result.getTx();
            panelsTelemetry.addData("tx", String.format("%.3f", tx));
            panelsTelemetry.addData("Status", Math.abs(tx) <= Tracking.txDeadzone ? "LOCKED ON TARGET" : "TRACKING");
        } else {
            panelsTelemetry.addData("Limelight", "NO TARGET");
        }
        panelsTelemetry.update();
    }

    private double currentServoPos() {
        return spindexerCurrentPos;
    }
}

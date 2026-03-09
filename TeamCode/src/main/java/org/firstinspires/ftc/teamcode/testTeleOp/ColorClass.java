package org.firstinspires.ftc.teamcode.testTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Color Test")
public class ColorClass extends LinearOpMode {

    TelemetryManager.TelemetryWrapper panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // =========================================================
    //  CONFIGURABLES
    // =========================================================

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
        public static boolean enableLEDs = true;
    }

    @Configurable
    public static class DistanceConfig {
        // Detection threshold in inches (SwyftRanger 20° formula: voltage*48.7 - 4.9)
        public static double detectionDistanceIn = 3.0;
    }

    @Configurable
    public static class SpindexerConfig {
        public static double initPosition = 0.5;
        public static double slot1 = 0.097;
        public static double slot2 = 0.287;
        public static double slot3 = 0.477;
        public static double fullRotation = 0.582;
        public static double transferDurationMs = 1000;
        // Time (ms) after servo move before sensors activate
        public static double settleTimeMs = 300;
        // Auto-advance spindexer after ball detected
        public static boolean autoAdvance = true;
        // Servo slew speed in servo-units per SECOND (loop-rate independent).
        // 5.0 = near-instant, 0.5 = one slot in ~0.5s, 0.2 = one slot in ~1.2s
        public static double servoSpeed = 5.0;
        // Speed during 360° transfer spin — lower = slower spin
        public static double transferServoSpeed = 0.5;
    }

    @Configurable
    public static class IntakeConfig {
        public static double intakePower = 1.0;
        public static double outtakePower = -1.0;
        public static double detectionDistanceIn = 3.0;
        public static double colorTimeoutMs = 2000.0;
        // Pause intake briefly after ball detected (ms)
        public static double pauseOnDetectMs = 100;
    }

    // =========================================================
    //  HARDWARE
    // =========================================================

    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private AnalogInput ranger;
    private DcMotorEx intake;
    private Servo spindexerServo1;
    private Servo spindexerServo2;

    // =========================================================
    //  STATE
    // =========================================================

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime slewTimer = new ElapsedTime();

    // Intake
    private boolean intakeToggled = false;
    private boolean lastCross = false;

    // Ball tracking
    private int detectedBalls = 0;
    private String[] ballSlots = new String[3];
    private boolean ballDetectedLastLoop = false;

    // Color detection
    private boolean awaitingColorDetect = false;
    private double colorDetectStartTime = 0;

    // Spindexer
    private int spindexerStep = -1;
    private double spindexerTargetPos = 0.5;
    private double spindexerCurrentPos = 0.5;
    private double spindexerMoveTime = 0;
    private boolean sensorsEnabled = false;
    private boolean transferActive = false;
    private double transferStartTime = 0;

    // Intake pause
    private double intakePauseUntil = 0;
    private boolean intakeActive = true;

    // Manual spindexer
    private boolean lastSquare = false;
    private boolean lastDpadDown = false;
    private boolean lastTriangle = false;
    private boolean lastCircle = false;

    // =========================================================
    //  MAIN
    // =========================================================

    @Override
    public void runOpMode() {
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
        ranger = hardwareMap.get(AnalogInput.class, "distanceSensor");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerServo1 = hardwareMap.get(Servo.class, "servo1");
        spindexerServo2 = hardwareMap.get(Servo.class, "servo2");

        spindexerStep = -1;
        setSpindexerPosition(SpindexerConfig.initPosition);
        setLEDs(false);
        for (int i = 0; i < 3; i++) ballSlots[i] = null;

        panelsTelemetry.update();
        telemetry.addData("Status", "Ready — press Start");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double nowMs = timer.milliseconds();

            updateSpindexerSlew();

            // Transfer timer
            if (transferActive && (nowMs - transferStartTime) >= SpindexerConfig.transferDurationMs) {
                transferActive = false;
                clearBalls();
                intakeToggled = false;
            }

            // Apply gain
            colorSensor1.setGain((float) ColorSensor1.gain);
            colorSensor2.setGain((float) ColorSensor2.gain);

            // --- Sensor gating: only enable when spindexer is settled ---
            if (spindexerSettled() && !sensorsEnabled) {
                sensorsEnabled = true;
            }
            setLEDs(sensorsEnabled && ColorSensor2.enableLEDs);

            // --- Cross (X): Toggle intake ---
            boolean cross = gamepad1.cross;
            if (cross && !lastCross) {
                intakeToggled = !intakeToggled;
                if (intakeToggled && spindexerStep == -1) {
                    spindexerStep = 0;
                    setSpindexerPosition(SpindexerConfig.slot1);
                }
            }
            lastCross = cross;

            // --- Outtake (dpad_right held) ---
            if (gamepad1.dpad_right) {
                intake.setPower(IntakeConfig.outtakePower);
            } else if (intakeToggled && intakeActive) {
                intake.setPower(IntakeConfig.intakePower);
            } else {
                intake.setPower(0);
            }

            // --- Intake pause timer ---
            if (nowMs > intakePauseUntil) intakeActive = true;

            // --- Ball detection (only when sensors enabled) ---
            boolean ballNow = sensorsEnabled && detectBall();
            boolean ballAdded = false;

            if (sensorsEnabled && intakeToggled && !awaitingColorDetect && intakeActive) {
                if (ballNow && !ballDetectedLastLoop && detectedBalls < 3) {
                    awaitingColorDetect = true;
                    colorDetectStartTime = nowMs;
                }
            }
            if (awaitingColorDetect && detectedBalls < 3) {
                String color = detectBallColor();
                boolean timedOut = (nowMs - colorDetectStartTime) > IntakeConfig.colorTimeoutMs;
                if (color != null || timedOut) {
                    commitBall((color != null) ? color : "purple");
                    ballAdded = true;
                }
            }

            if (ballAdded) {
                intakePauseUntil = nowMs + IntakeConfig.pauseOnDetectMs;
                intakeActive = false;
            }
            ballDetectedLastLoop = ballNow;

            // --- Square: manually advance spindexer ---
            boolean square = gamepad1.square;
            if (square && !lastSquare) {
                if (detectedBalls < 3) {
                    addBall("unknown");
                    detectedBalls++;
                    advanceSpindexer();
                } else {
                    advanceSpindexerForce();
                }
            }
            lastSquare = square;

            // --- Dpad Down: reverse one slot ---
            boolean dpadDown = gamepad1.dpad_down;
            if (dpadDown && !lastDpadDown) {
                reverseSpindexer();
                if (detectedBalls > 0) {
                    ballSlots[0] = ballSlots[1];
                    ballSlots[1] = ballSlots[2];
                    ballSlots[2] = null;
                    detectedBalls--;
                }
            }
            lastDpadDown = dpadDown;

            // --- Triangle: reset everything ---
            boolean triangle = gamepad1.triangle;
            if (triangle && !lastTriangle) {
                clearBalls();
                intakeToggled = false;
                spindexerStep = -1;
                setSpindexerPosition(SpindexerConfig.initPosition);
            }
            lastTriangle = triangle;

            // --- Circle: shoot (rotate fullRotation using transferServoSpeed) ---
            boolean circle = gamepad1.circle;
            if (circle && !lastCircle && !transferActive) {
                double startPos = (spindexerStep >= 0) ? getSlotPosition(spindexerStep) : SpindexerConfig.slot1;
                setSpindexerPosition(clamp(startPos + SpindexerConfig.fullRotation));
                transferActive = true;
                transferStartTime = nowMs;
            }
            lastCircle = circle;

            // --- Triggers: manual servo control ---
            double manual = 0;
            if (gamepad1.right_trigger > 0.05) manual += gamepad1.right_trigger * 0.005;
            if (gamepad1.left_trigger > 0.05)  manual -= gamepad1.left_trigger * 0.005;
            if (manual != 0) {
                double pos = currentServoPos() + manual;
                spindexerServo1.setPosition(clamp(pos));
                spindexerServo2.setPosition(clamp(1.0 - pos));
                // Don't reset spindexerMoveTime for manual — no settle needed for tiny moves
            }

            // --- Read both sensors with their own gains ---
            colorSensor1.setGain((float) ColorSensor1.gain);
            colorSensor2.setGain((float) ColorSensor2.gain);
            NormalizedRGBA raw1 = colorSensor1.getNormalizedColors();
            NormalizedRGBA raw2 = colorSensor2.getNormalizedColors();
            float a1 = Math.max(raw1.alpha, 1e-6f);
            float a2 = Math.max(raw2.alpha, 1e-6f);
            double cs1R = raw1.red / a1,  cs1G = raw1.green / a1,  cs1B = raw1.blue / a1;
            double cs2R = raw2.red / a2,  cs2G = raw2.green / a2,  cs2B = raw2.blue / a2;
            boolean cs1Green  = cs1IsGreen(cs1R, cs1G, cs1B);
            boolean cs2Green  = cs2IsGreen(cs2R, cs2G, cs2B);
            boolean cs1Purple = cs1IsPurple(cs1R, cs1G, cs1B);
            boolean cs2Purple = cs2IsPurple(cs2R, cs2G, cs2B);
            String detected = (cs1Green || cs2Green) ? "green"
                            : (cs1Purple || cs2Purple) ? "purple" : null;

            // --- Distance ---
            double voltage   = ranger.getVoltage();
            double distIn    = (voltage * 48.7) - 4.9;
            boolean ballHere = distIn <= DistanceConfig.detectionDistanceIn;

            // --- Driver station ---
            telemetry.addData("DETECTED", detected != null ? detected.toUpperCase() : "NONE");
            telemetry.addData("CS1", String.format("R=%.3f G=%.3f B=%.3f  %s", cs1R, cs1G, cs1B, cs1Green ? "GREEN" : cs1Purple ? "PURPLE" : "none"));
            telemetry.addData("CS2", String.format("R=%.3f G=%.3f B=%.3f  %s", cs2R, cs2G, cs2B, cs2Green ? "GREEN" : cs2Purple ? "PURPLE" : "none"));
            telemetry.addData("Gain CS1/CS2", ColorSensor1.gain + " / " + ColorSensor2.gain);
            telemetry.addData("Distance (in)", String.format("%.2f  [thresh=%.2f]  %s", distIn, DistanceConfig.detectionDistanceIn, ballHere ? "BALL" : "none"));
            telemetry.addData("Ranger Voltage", String.format("%.3fV", voltage));
            telemetry.addData("Intake", intakeToggled ? "ON" : "OFF");
            telemetry.addData("Balls", detectedBalls + "/3  " + formatBallSlots());
            telemetry.addData("Spindexer", (spindexerStep == -1 ? "INIT" : "Slot " + (spindexerStep + 1)) + "  pos=" + String.format("%.3f", currentServoPos()));
            telemetry.addData("Controls", "X intake  [] add  O shoot  /\\ reset  v reverse  triggers manual");
            telemetry.update();

            // --- Panels dashboard ---
            panelsTelemetry.addData("=== DETECTED ===", detected != null ? detected.toUpperCase() : "NONE");
            panelsTelemetry.addData("", "");

            panelsTelemetry.addData("=== COLOR SENSOR 1 ===", "gain=" + ColorSensor1.gain);
            panelsTelemetry.addData("CS1 Norm R", String.format("%.4f  [%.2f–%.2f]  %s", cs1R, ColorSensor1.greenRMin, ColorSensor1.greenRMax, (cs1R >= ColorSensor1.greenRMin && cs1R <= ColorSensor1.greenRMax) ? "OK" : "--"));
            panelsTelemetry.addData("CS1 Norm G", String.format("%.4f  [%.2f–%.2f]  %s", cs1G, ColorSensor1.greenGMin, ColorSensor1.greenGMax, (cs1G >= ColorSensor1.greenGMin && cs1G <= ColorSensor1.greenGMax) ? "OK" : "--"));
            panelsTelemetry.addData("CS1 Norm B", String.format("%.4f  [%.2f–%.2f]  %s", cs1B, ColorSensor1.greenBMin, ColorSensor1.greenBMax, (cs1B >= ColorSensor1.greenBMin && cs1B <= ColorSensor1.greenBMax) ? "OK" : "--"));
            panelsTelemetry.addData("CS1 Alpha", String.format("%.4f", (double) raw1.alpha));
            panelsTelemetry.addData("CS1 Result", cs1Green ? "GREEN" : cs1Purple ? "PURPLE" : "none");
            panelsTelemetry.addData("", "");

            panelsTelemetry.addData("=== COLOR SENSOR 2 ===", "gain=" + ColorSensor2.gain);
            panelsTelemetry.addData("CS2 Norm R", String.format("%.4f  [%.2f–%.2f]  %s", cs2R, ColorSensor2.greenRMin, ColorSensor2.greenRMax, (cs2R >= ColorSensor2.greenRMin && cs2R <= ColorSensor2.greenRMax) ? "OK" : "--"));
            panelsTelemetry.addData("CS2 Norm G", String.format("%.4f  [%.2f–%.2f]  %s", cs2G, ColorSensor2.greenGMin, ColorSensor2.greenGMax, (cs2G >= ColorSensor2.greenGMin && cs2G <= ColorSensor2.greenGMax) ? "OK" : "--"));
            panelsTelemetry.addData("CS2 Norm B", String.format("%.4f  [%.2f–%.2f]  %s", cs2B, ColorSensor2.greenBMin, ColorSensor2.greenBMax, (cs2B >= ColorSensor2.greenBMin && cs2B <= ColorSensor2.greenBMax) ? "OK" : "--"));
            panelsTelemetry.addData("CS2 Alpha", String.format("%.4f", (double) raw2.alpha));
            panelsTelemetry.addData("CS2 Result", cs2Green ? "GREEN" : cs2Purple ? "PURPLE" : "none");
            panelsTelemetry.addData("", "");

            panelsTelemetry.addData("=== DISTANCE SENSOR ===", "");
            panelsTelemetry.addData("Voltage", String.format("%.4fV", voltage));
            panelsTelemetry.addData("Distance (in)", String.format("%.3f", distIn));
            panelsTelemetry.addData("Threshold (in)", DistanceConfig.detectionDistanceIn);
            panelsTelemetry.addData("Ball Detected", ballHere ? "YES" : "NO");
            panelsTelemetry.addData("", "");

            panelsTelemetry.addData("=== STATE ===", "");
            panelsTelemetry.addData("Intake", intakeToggled ? "ON" : "OFF");
            panelsTelemetry.addData("Balls", detectedBalls + "/3  " + formatBallSlots());
            panelsTelemetry.addData("Spindexer", spindexerStep == -1 ? "INIT" : "Slot " + (spindexerStep + 1));
            panelsTelemetry.addData("Servo pos", String.format("%.4f", currentServoPos()));
            panelsTelemetry.update();
        }

        intake.setPower(0);
        setLEDs(false);
        setSpindexerPosition(SpindexerConfig.initPosition);
    }

    // =========================================================
    //  COLOR DETECTION
    // =========================================================

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

    /** Check each sensor with its own ranges — either passing = detected. */
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

    private boolean detectBall() {
        return ((ranger.getVoltage() * 48.7) - 4.9) <= DistanceConfig.detectionDistanceIn;
    }

    // =========================================================
    //  SPINDEXER
    // =========================================================

    private void setSpindexerPosition(double pos) {
        spindexerTargetPos = clamp(pos);
        spindexerMoveTime = timer.milliseconds();
        sensorsEnabled = false;
    }

    private void updateSpindexerSlew() {
        double elapsed = slewTimer.seconds();
        slewTimer.reset();
        double speed = transferActive ? SpindexerConfig.transferServoSpeed : SpindexerConfig.servoSpeed;
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

    private boolean spindexerSettled() {
        return (timer.milliseconds() - spindexerMoveTime) >= SpindexerConfig.settleTimeMs;
    }

    private double getSlotPosition(int slot) {
        switch (slot) {
            case 0:  return SpindexerConfig.slot1;
            case 1:  return SpindexerConfig.slot2;
            case 2:  return SpindexerConfig.slot3;
            default: return SpindexerConfig.initPosition;
        }
    }

    /** Advance to the next empty slot (skips filled). */
    private void advanceSpindexer() {
        if (!SpindexerConfig.autoAdvance) {
            advanceSpindexerForce();
            return;
        }
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
        advanceSpindexerForce();
    }

    /** Advance one slot unconditionally (capped at slot3). */
    private void advanceSpindexerForce() {
        if (spindexerStep < 2) {
            spindexerStep++;
            setSpindexerPosition(getSlotPosition(spindexerStep));
        }
    }

    private void reverseSpindexer() {
        if (spindexerStep > 0) {
            spindexerStep--;
            setSpindexerPosition(getSlotPosition(spindexerStep));
        } else {
            spindexerStep = -1;
            setSpindexerPosition(SpindexerConfig.initPosition);
        }
    }

    // =========================================================
    //  BALL TRACKING
    // =========================================================

    private void addBall(String color) {
        ballSlots[0] = ballSlots[1];
        ballSlots[1] = ballSlots[2];
        ballSlots[2] = color;
    }

    private void commitBall(String color) {
        addBall(color);
        detectedBalls++;
        advanceSpindexer();
        awaitingColorDetect = false;
        if (detectedBalls == 3) {
            gamepad1.rumble(0.5, 0.5, 300);
        }
    }

    private void clearBalls() {
        for (int i = 0; i < 3; i++) ballSlots[i] = null;
        detectedBalls = 0;
        ballDetectedLastLoop = false;
        awaitingColorDetect = false;
        spindexerStep = 0;
        setSpindexerPosition(SpindexerConfig.slot1);
    }

    private String formatBallSlots() {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < 3; i++) {
            sb.append(ballSlots[i] == null ? "empty" : ballSlots[i]);
            if (i < 2) sb.append(", ");
        }
        return sb.append("]").toString();
    }

    // =========================================================
    //  HELPERS
    // =========================================================

    private double currentServoPos() {
        return (spindexerStep == -1) ? SpindexerConfig.initPosition : getSlotPosition(spindexerStep);
    }

    private double clamp(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }

    private void setLEDs(boolean on) {
        if (colorSensor1 instanceof SwitchableLight) ((SwitchableLight) colorSensor1).enableLight(on);
        if (colorSensor2 instanceof SwitchableLight) ((SwitchableLight) colorSensor2).enableLight(on);
    }
}

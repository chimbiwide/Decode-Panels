package org.firstinspires.ftc.teamcode.testTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Test")
public class ColorClass extends LinearOpMode {

    TelemetryManager.TelemetryWrapper panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // =========================================================
    //  CONFIGURABLES
    // =========================================================

    @Configurable
    public static class ColorConfig {
        public static double gain = 8.0;

        // --- Green thresholds (alpha-normalized) ---
        public static double greenMin = 0.35;
        public static double greenRedMax = 0.33;
        public static double greenBlueMax = 0.33;

        // --- Purple thresholds (alpha-normalized) ---
        public static double purpleRedMin = 0.30;
        public static double purpleBlueMin = 0.30;
        public static double purpleGreenMax = 0.33;

        public static boolean enableLEDs = true;
    }

    @Configurable
    public static class SpindexerConfig {
        public static double initPosition = 0.5;
        public static double slot1 = 0.097;
        public static double slot2 = 0.287;
        public static double slot3 = 0.477;
        public static double stepSize = 0.2353;
        public static double shootRotation = 0.4706;
        // Time (ms) after servo move before sensors activate
        public static double settleTimeMs = 300;
        // Auto-advance spindexer after ball detected
        public static boolean autoAdvance = true;
    }

    @Configurable
    public static class IntakeConfig {
        public static double intakePower = 1.0;
        public static double outtakePower = -1.0;
        public static double detectionDistanceMM = 50.0;
        public static double colorTimeoutMs = 2000.0;
        // Pause intake briefly after ball detected (ms)
        public static double pauseOnDetectMs = 100;
    }

    // =========================================================
    //  HARDWARE
    // =========================================================

    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private DistanceSensor distanceSensor;
    private DcMotorEx intake;
    private Servo spindexerServo1;
    private Servo spindexerServo2;

    // =========================================================
    //  STATE
    // =========================================================

    private final ElapsedTime timer = new ElapsedTime();

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
    private double spindexerMoveTime = 0;
    private boolean sensorsEnabled = false;

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
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerServo1 = hardwareMap.get(Servo.class, "spindexerServo1");
        spindexerServo2 = hardwareMap.get(Servo.class, "spindexerServo2");

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

            // Apply gain
            colorSensor1.setGain((float) ColorConfig.gain);
            colorSensor2.setGain((float) ColorConfig.gain);

            // --- Sensor gating: only enable when spindexer is settled ---
            if (spindexerSettled() && !sensorsEnabled) {
                sensorsEnabled = true;
            }
            setLEDs(sensorsEnabled && ColorConfig.enableLEDs);

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

            // --- Circle: shoot (rotate full 240°) ---
            boolean circle = gamepad1.circle;
            if (circle && !lastCircle && detectedBalls > 0) {
                double startPos = (spindexerStep >= 0) ? getSlotPosition(spindexerStep) : SpindexerConfig.slot1;
                setSpindexerPosition(clamp(startPos + SpindexerConfig.shootRotation));
                clearBalls();
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

            // --- Read colors for telemetry ---
            NormalizedRGBA raw1 = colorSensor1.getNormalizedColors();
            NormalizedRGBA raw2 = colorSensor2.getNormalizedColors();
            float avgR = (raw1.red + raw2.red) / 2f;
            float avgG = (raw1.green + raw2.green) / 2f;
            float avgB = (raw1.blue + raw2.blue) / 2f;
            float avgA = (raw1.alpha + raw2.alpha) / 2f;
            double normR = (avgA > 0) ? avgR / avgA : 0;
            double normG = (avgA > 0) ? avgG / avgA : 0;
            double normB = (avgA > 0) ? avgB / avgA : 0;
            String detected = detectColor(normR, normG, normB);
            double distMM = distanceSensor.getDistance(DistanceUnit.MM);
            float a1 = Math.max(raw1.alpha, 1e-6f);
            float a2 = Math.max(raw2.alpha, 1e-6f);

            // --- Driver station ---
            telemetry.addData("DETECTED", detected != null ? detected.toUpperCase() : "NONE");
            telemetry.addData("Intake", intakeToggled ? "ON" : "OFF");
            telemetry.addData("Balls", detectedBalls + "/3  " + formatBallSlots());
            telemetry.addData("Sensors", sensorsEnabled ? "ON" : "WAITING");
            telemetry.addData("Spindexer",
                    (spindexerStep == -1 ? "INIT" : "Slot " + (spindexerStep + 1))
                    + "  pos=" + String.format("%.3f", currentServoPos()));
            telemetry.addData("Distance (mm)", String.format("%.1f", distMM));
            telemetry.addData("Norm R/G/B", "%.3f / %.3f / %.3f", normR, normG, normB);
            telemetry.addData("Gain", ColorConfig.gain);
            telemetry.addData("", "");
            telemetry.addData("Controls", "X intake  [] add  O shoot  /\\ reset  v reverse  triggers manual");
            telemetry.update();

            // --- Panels dashboard ---
            panelsTelemetry.addData("=== STATE ===", "");
            panelsTelemetry.addData("Detected", detected != null ? detected.toUpperCase() : "NONE");
            panelsTelemetry.addData("Intake", intakeToggled ? "ON" : "OFF");
            panelsTelemetry.addData("Balls", detectedBalls + "/3  " + formatBallSlots());
            panelsTelemetry.addData("Sensors", sensorsEnabled ? "ON" : "WAITING");
            panelsTelemetry.addData("Spindexer Slot", spindexerStep == -1 ? "INIT" : "Slot " + (spindexerStep + 1));
            panelsTelemetry.addData("Servo1 pos", String.format("%.4f", currentServoPos()));
            panelsTelemetry.addData("Distance (mm)", String.format("%.1f", distMM));
            panelsTelemetry.addData("Ball at sensor", distMM <= IntakeConfig.detectionDistanceMM ? "YES" : "NO");
            panelsTelemetry.addData("", "");

            panelsTelemetry.addData("=== COLOR (normalized) ===", "");
            panelsTelemetry.addData("Norm R", String.format("%.4f", normR));
            panelsTelemetry.addData("Norm G", String.format("%.4f", normG));
            panelsTelemetry.addData("Norm B", String.format("%.4f", normB));
            panelsTelemetry.addData("Avg Alpha", String.format("%.4f", (double) avgA));
            panelsTelemetry.addData("Gain", ColorConfig.gain);
            panelsTelemetry.addData("", "");

            panelsTelemetry.addData("=== GREEN CHECK ===", "");
            panelsTelemetry.addData("normG > greenMin?",
                    String.format("%.3f > %.3f = %s", normG, ColorConfig.greenMin, normG > ColorConfig.greenMin));
            panelsTelemetry.addData("normR < greenRedMax?",
                    String.format("%.3f < %.3f = %s", normR, ColorConfig.greenRedMax, normR < ColorConfig.greenRedMax));
            panelsTelemetry.addData("normB < greenBlueMax?",
                    String.format("%.3f < %.3f = %s", normB, ColorConfig.greenBlueMax, normB < ColorConfig.greenBlueMax));
            panelsTelemetry.addData("", "");
            panelsTelemetry.addData("=== PURPLE CHECK ===", "");
            panelsTelemetry.addData("normR > purpleRedMin?",
                    String.format("%.3f > %.3f = %s", normR, ColorConfig.purpleRedMin, normR > ColorConfig.purpleRedMin));
            panelsTelemetry.addData("normB > purpleBlueMin?",
                    String.format("%.3f > %.3f = %s", normB, ColorConfig.purpleBlueMin, normB > ColorConfig.purpleBlueMin));
            panelsTelemetry.addData("normG < purpleGreenMax?",
                    String.format("%.3f < %.3f = %s", normG, ColorConfig.purpleGreenMax, normG < ColorConfig.purpleGreenMax));
            panelsTelemetry.addData("", "");
            panelsTelemetry.addData("=== RAW PER SENSOR ===", "");
            panelsTelemetry.addData("CS1 R/G/B/A", String.format("%.3f / %.3f / %.3f / %.3f",
                    (double) raw1.red, (double) raw1.green, (double) raw1.blue, (double) raw1.alpha));
            panelsTelemetry.addData("CS1 norm R/G/B", String.format("%.3f / %.3f / %.3f",
                    (double) (raw1.red / a1), (double) (raw1.green / a1), (double) (raw1.blue / a1)));
            panelsTelemetry.addData("CS2 R/G/B/A", String.format("%.3f / %.3f / %.3f / %.3f",
                    (double) raw2.red, (double) raw2.green, (double) raw2.blue, (double) raw2.alpha));
            panelsTelemetry.addData("CS2 norm R/G/B", String.format("%.3f / %.3f / %.3f",
                    (double) (raw2.red / a2), (double) (raw2.green / a2), (double) (raw2.blue / a2)));
            panelsTelemetry.update();
        }

        intake.setPower(0);
        setLEDs(false);
        setSpindexerPosition(SpindexerConfig.initPosition);
    }

    // =========================================================
    //  COLOR DETECTION
    // =========================================================

    public static String detectColor(double normR, double normG, double normB) {
        if (normG > ColorConfig.greenMin && normR < ColorConfig.greenRedMax && normB < ColorConfig.greenBlueMax) {
            return "green";
        }
        if (normR > ColorConfig.purpleRedMin && normB > ColorConfig.purpleBlueMin && normG < ColorConfig.purpleGreenMax) {
            return "purple";
        }
        return null;
    }

    private String detectBallColor() {
        NormalizedRGBA c1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA c2 = colorSensor2.getNormalizedColors();
        float avgR = (c1.red + c2.red) / 2f;
        float avgG = (c1.green + c2.green) / 2f;
        float avgB = (c1.blue + c2.blue) / 2f;
        float avgA = (c1.alpha + c2.alpha) / 2f;
        double normR = (avgA > 0) ? avgR / avgA : 0;
        double normG = (avgA > 0) ? avgG / avgA : 0;
        double normB = (avgA > 0) ? avgB / avgA : 0;
        return detectColor(normR, normG, normB);
    }

    private boolean detectBall() {
        return distanceSensor.getDistance(DistanceUnit.MM) <= IntakeConfig.detectionDistanceMM;
    }

    // =========================================================
    //  SPINDEXER
    // =========================================================

    private void setSpindexerPosition(double pos) {
        pos = clamp(pos);
        spindexerServo1.setPosition(pos);
        spindexerServo2.setPosition(1.0 - pos);
        spindexerMoveTime = timer.milliseconds();
        sensorsEnabled = false;
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

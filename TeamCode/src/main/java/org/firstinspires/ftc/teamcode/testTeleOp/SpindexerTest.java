package org.firstinspires.ftc.teamcode.testTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Spindexer Test")
public class SpindexerTest extends LinearOpMode {

    TelemetryManager.TelemetryWrapper panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // =========================================================
    //  CONFIGURABLES
    // =========================================================

    @Configurable
    public static class ServoConfig {
        // Servo output: 255° range, 2:1 gear = 510° total output
        public static double servoRangeDeg = 255.0;
        public static double gearRatio = 2.0;
        // Computed: totalOutputDeg = servoRangeDeg * gearRatio = 510

        // Spindexer geometry
        public static double degreesPerSlot = 120.0;  // 360° / 3 slots
        public static int totalSlots = 3;

        // Servo position per slot step: degreesPerSlot / (servoRangeDeg * gearRatio)
        // 120 / 510 = 0.2353
        public static double stepSize = 0.2353;

        // Full shoot rotation: 2 steps = 240° = 0.4706 servo pos
        // (fires slot1 -> slot2 -> slot3 across the shooter)
        public static double shootRotation = 0.4706;

        // Starting servo position (where slot1 aligns with intake/shooter)
        public static double startPosition = 0.1;

        // Manual trigger control speed (servo units per loop)
        public static double manualSpeed = 0.005;
    }

    @Configurable
    public static class TransferMotorConfig {
        public static double shootPower = 1.0;
        public static double shootDurationMs = 3300;
    }

    // =========================================================
    //  HARDWARE
    // =========================================================

    private Servo spindexerServo1;
    private Servo spindexerServo2;

    // =========================================================
    //  STATE
    // =========================================================

    private double currentPosition;
    private int currentSlot = 0;  // 0 = slot1, 1 = slot2, 2 = slot3

    // Shooting
    private boolean shootActive = false;
    private double shootStartTime = 0;
    private double shootTargetPos = 0;

    // Edge detection
    private boolean lastSquare = false;
    private boolean lastCircle = false;
    private boolean lastTriangle = false;
    private boolean lastDpadDown = false;

    private final ElapsedTime timer = new ElapsedTime();

    // =========================================================
    //  MAIN
    // =========================================================

    @Override
    public void runOpMode() {
        spindexerServo1 = hardwareMap.get(Servo.class, "spindexerServo1");
        spindexerServo2 = hardwareMap.get(Servo.class, "spindexerServo2");

        currentPosition = ServoConfig.startPosition;
        setServoPosition(currentPosition);

        panelsTelemetry.update();
        telemetry.addData("Status", "Ready — press Start");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {

            // --- Square: advance one slot (120°) ---
            boolean square = gamepad1.square;
            if (square && !lastSquare && !shootActive) {
                currentPosition += ServoConfig.stepSize;
                currentPosition = clamp(currentPosition);
                currentSlot = (currentSlot + 1) % ServoConfig.totalSlots;
                setServoPosition(currentPosition);
            }
            lastSquare = square;

            // --- Dpad Down: reverse one slot (-120°) ---
            boolean dpadDown = gamepad1.dpad_down;
            if (dpadDown && !lastDpadDown && !shootActive) {
                currentPosition -= ServoConfig.stepSize;
                currentPosition = clamp(currentPosition);
                currentSlot = ((currentSlot - 1) % ServoConfig.totalSlots + ServoConfig.totalSlots) % ServoConfig.totalSlots;
                setServoPosition(currentPosition);
            }
            lastDpadDown = dpadDown;

            // --- Circle: shoot (rotate 240° from current position) ---
            boolean circle = gamepad1.circle;
            if (circle && !lastCircle && !shootActive) {
                shootTargetPos = clamp(currentPosition + ServoConfig.shootRotation);
                shootActive = true;
                shootStartTime = timer.milliseconds();
                setServoPosition(shootTargetPos);
            }
            lastCircle = circle;

            // --- Shoot timer ---
            if (shootActive) {
                double elapsed = timer.milliseconds() - shootStartTime;
                if (elapsed >= TransferMotorConfig.shootDurationMs) {
                    shootActive = false;
                    currentPosition = shootTargetPos;
                    currentSlot = 0;
                }
            }

            // --- Triangle: reset to start position ---
            boolean triangle = gamepad1.triangle;
            if (triangle && !lastTriangle && !shootActive) {
                currentPosition = ServoConfig.startPosition;
                currentSlot = 0;
                setServoPosition(currentPosition);
            }
            lastTriangle = triangle;

            // --- Triggers: manual servo control ---
            if (!shootActive) {
                double manual = 0;
                if (gamepad1.right_trigger > 0.05) manual += gamepad1.right_trigger * ServoConfig.manualSpeed;
                if (gamepad1.left_trigger > 0.05)  manual -= gamepad1.left_trigger * ServoConfig.manualSpeed;
                if (manual != 0) {
                    currentPosition = clamp(currentPosition + manual);
                    setServoPosition(currentPosition);
                }
            }

            // --- Telemetry ---
            double totalOutputDeg = ServoConfig.servoRangeDeg * ServoConfig.gearRatio;
            double currentDeg = currentPosition * totalOutputDeg;

            telemetry.addData("Slot", (currentSlot + 1) + " / " + ServoConfig.totalSlots);
            telemetry.addData("Servo Pos", "%.4f", currentPosition);
            telemetry.addData("Output Deg", "%.1f°", currentDeg);
            telemetry.addData("Shoot", shootActive ? "ACTIVE" : "IDLE");
            telemetry.addData("Step Size", "%.4f  (%.1f°)", ServoConfig.stepSize, ServoConfig.stepSize * totalOutputDeg);
            telemetry.addData("Shoot Rotation", "%.4f  (%.1f°)", ServoConfig.shootRotation, ServoConfig.shootRotation * totalOutputDeg);
            telemetry.addData("Controls", "[] step  O shoot  /\\ reset  triggers manual");
            telemetry.update();

            panelsTelemetry.addData("=== SPINDEXER TEST ===", "");
            panelsTelemetry.addData("Slot", (currentSlot + 1) + " / " + ServoConfig.totalSlots);
            panelsTelemetry.addData("Servo Pos", String.format("%.4f", currentPosition));
            panelsTelemetry.addData("Servo2 Pos", String.format("%.4f", 1.0 - currentPosition));
            panelsTelemetry.addData("Output Degrees", String.format("%.1f°", currentDeg));
            panelsTelemetry.addData("Shooting", shootActive
                    ? "ACTIVE  " + (int) Math.max(0, TransferMotorConfig.shootDurationMs - (timer.milliseconds() - shootStartTime)) + "ms left"
                    : "IDLE");
            panelsTelemetry.addData("", "");
            panelsTelemetry.addData("=== COMPUTED ===", "");
            panelsTelemetry.addData("Total Output", String.format("%.0f°", totalOutputDeg));
            panelsTelemetry.addData("Step", String.format("%.4f pos = %.1f°", ServoConfig.stepSize, ServoConfig.stepSize * totalOutputDeg));
            panelsTelemetry.addData("Shoot Spin", String.format("%.4f pos = %.1f°", ServoConfig.shootRotation, ServoConfig.shootRotation * totalOutputDeg));
            panelsTelemetry.addData("Room Left", String.format("%.4f pos = %.1f°", 1.0 - currentPosition, (1.0 - currentPosition) * totalOutputDeg));
            panelsTelemetry.update();
        }

        setServoPosition(ServoConfig.startPosition);
    }

    // =========================================================
    //  HELPERS
    // =========================================================

    private void setServoPosition(double pos) {
        pos = clamp(pos);
        spindexerServo1.setPosition(pos);
        spindexerServo2.setPosition(1.0 - pos);
    }

    private double clamp(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }
}

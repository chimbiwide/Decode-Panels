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

        // Full 360° = 0.5653 servo units. Step per slot = 0.5653 / 3 = 0.1884
        public static double stepSize = 0.1884;

        // Full 360° shoot rotation
        public static double shootRotation = 0.5653;

        // Starting servo position = slot1
        public static double startPosition = 0.097;

        // Trigger manual speed (servo units per second)
        public static double manualSpeed = 0.3;

        // Bumper manual speed (servo units per second) — held continuously
        public static double bumperSpeed = 0.15;

        // Servo slew speed in servo-units per SECOND (time-based, loop-rate independent).
        // One slot step = 0.2353 units.
        // 0.2 = one slot in ~1.2s, 0.5 = one slot in ~0.5s, 5.0 = near-instant.
        public static double servoSpeed = 5.0;

        // If servo2 is mechanically mirrored, set true so it goes opposite to servo1.
        // true  → servo2 = 1.0 - servo1  (they push in opposite directions — correct)
        // false → servo2 = servo1         (same direction — use if wired identically)
        public static boolean servo2Mirrored = true;
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

    private double currentPosition;   // actual servo position (slewes toward target)
    private double targetPosition;    // commanded target position
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
    private final ElapsedTime slewTimer = new ElapsedTime();
    private double lastLoopTime = 0;

    // =========================================================
    //  MAIN
    // =========================================================

    @Override
    public void runOpMode() {
        spindexerServo1 = hardwareMap.get(Servo.class, "servo1");
        spindexerServo2 = hardwareMap.get(Servo.class, "servo2");

        currentPosition = ServoConfig.startPosition;
        targetPosition = currentPosition;
        setServoPosition(currentPosition);

        panelsTelemetry.update();
        telemetry.addData("Status", "Ready — press Start");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double now = timer.seconds();
            double dt = Math.max(now - lastLoopTime, 1e-3);
            lastLoopTime = now;

            // --- Square: advance one slot ---
            boolean square = gamepad1.square;
            if (square && !lastSquare && !shootActive) {
                targetPosition = clamp(targetPosition + ServoConfig.stepSize);
                currentSlot = (currentSlot + 1) % ServoConfig.totalSlots;
            }
            lastSquare = square;

            // --- Dpad Down: reverse one slot ---
            boolean dpadDown = gamepad1.dpad_down;
            if (dpadDown && !lastDpadDown && !shootActive) {
                targetPosition = clamp(targetPosition - ServoConfig.stepSize);
                currentSlot = ((currentSlot - 1) % ServoConfig.totalSlots + ServoConfig.totalSlots) % ServoConfig.totalSlots;
            }
            lastDpadDown = dpadDown;

            // --- Circle: shoot (rotate 240° from current position) ---
            boolean circle = gamepad1.circle;
            if (circle && !lastCircle && !shootActive) {
                shootTargetPos = clamp(currentPosition + ServoConfig.shootRotation);
                targetPosition = shootTargetPos;
                shootActive = true;
                shootStartTime = timer.milliseconds();
            }
            lastCircle = circle;

            // --- Shoot timer ---
            if (shootActive) {
                double elapsed = timer.milliseconds() - shootStartTime;
                if (elapsed >= TransferMotorConfig.shootDurationMs) {
                    shootActive = false;
                    currentSlot = 0;
                }
            }

            // --- Triangle: reset to start position ---
            boolean triangle = gamepad1.triangle;
            if (triangle && !lastTriangle && !shootActive) {
                targetPosition = ServoConfig.startPosition;
                currentSlot = 0;
            }
            lastTriangle = triangle;

            // --- Bumpers: manual continuous move (held, configurable speed, time-based) ---
            if (!shootActive) {
                if (gamepad1.right_bumper) {
                    targetPosition = clamp(targetPosition + ServoConfig.bumperSpeed * dt);
                } else if (gamepad1.left_bumper) {
                    targetPosition = clamp(targetPosition - ServoConfig.bumperSpeed * dt);
                }
            }

            // --- Triggers: manual servo control (time-based) ---
            if (!shootActive) {
                double manual = 0;
                if (gamepad1.right_trigger > 0.05) manual += gamepad1.right_trigger * ServoConfig.manualSpeed * dt;
                if (gamepad1.left_trigger > 0.05)  manual -= gamepad1.left_trigger * ServoConfig.manualSpeed * dt;
                if (manual != 0) {
                    targetPosition = clamp(targetPosition + manual);
                }
            }

            // --- Slew using dedicated timer (loop-rate independent) ---
            double elapsed = slewTimer.seconds();
            slewTimer.reset();
            double maxStep = ServoConfig.servoSpeed * elapsed;
            double error = targetPosition - currentPosition;
            if (Math.abs(error) <= maxStep) {
                currentPosition = targetPosition;
            } else {
                currentPosition += Math.signum(error) * maxStep;
            }
            setServoPosition(currentPosition);

            // --- Telemetry ---
            double totalOutputDeg = ServoConfig.servoRangeDeg * ServoConfig.gearRatio;
            double currentDeg = currentPosition * totalOutputDeg;

            double servo2Pos = ServoConfig.servo2Mirrored ? (1.0 - currentPosition) : currentPosition;

            telemetry.addData("Slot", (currentSlot + 1) + " / " + ServoConfig.totalSlots);
            telemetry.addData("Servo1 pos", String.format("%.4f", currentPosition));
            telemetry.addData("Servo2 pos", String.format("%.4f  (%s)", servo2Pos, ServoConfig.servo2Mirrored ? "mirrored" : "same"));
            telemetry.addData("Target", String.format("%.4f", targetPosition));
            telemetry.addData("Output Deg", String.format("%.1f°", currentDeg));
            telemetry.addData("Shoot", shootActive ? "ACTIVE" : "IDLE");
            telemetry.addData("Controls", "[] step  v reverse  O shoot  /\\ reset  bumpers/triggers manual");
            telemetry.update();

            panelsTelemetry.addData("=== SPINDEXER TEST ===", "");
            panelsTelemetry.addData("Slot", (currentSlot + 1) + " / " + ServoConfig.totalSlots);
            panelsTelemetry.addData("", "");
            panelsTelemetry.addData("Servo1 pos (actual)", String.format("%.4f", currentPosition));
            panelsTelemetry.addData("Servo1 pos (target)", String.format("%.4f", targetPosition));
            panelsTelemetry.addData("Servo2 pos (actual)", String.format("%.4f", servo2Pos));
            panelsTelemetry.addData("Servo2 mode", ServoConfig.servo2Mirrored ? "MIRRORED (1.0 - servo1)" : "SAME as servo1");
            panelsTelemetry.addData("", "");
            panelsTelemetry.addData("Output Degrees", String.format("%.1f°", currentDeg));
            panelsTelemetry.addData("Servo Speed", String.format("%.3f u/s", ServoConfig.servoSpeed));
            panelsTelemetry.addData("Bumper Speed", String.format("%.3f u/s", ServoConfig.bumperSpeed));
            panelsTelemetry.addData("Trigger Speed", String.format("%.3f u/s", ServoConfig.manualSpeed));
            panelsTelemetry.addData("Shooting", shootActive
                    ? "ACTIVE  " + (int) Math.max(0, TransferMotorConfig.shootDurationMs - (timer.milliseconds() - shootStartTime)) + "ms left"
                    : "IDLE");
            panelsTelemetry.addData("", "");
            panelsTelemetry.addData("=== COMPUTED ===", "");
            panelsTelemetry.addData("Total Output", String.format("%.0f°", totalOutputDeg));
            panelsTelemetry.addData("Step", String.format("%.4f = %.1f°", ServoConfig.stepSize, ServoConfig.stepSize * totalOutputDeg));
            panelsTelemetry.addData("Shoot Spin", String.format("%.4f = %.1f°", ServoConfig.shootRotation, ServoConfig.shootRotation * totalOutputDeg));
            panelsTelemetry.addData("Room Left", String.format("%.4f = %.1f°", 1.0 - currentPosition, (1.0 - currentPosition) * totalOutputDeg));
            panelsTelemetry.update();
        }

        setServoPosition(ServoConfig.startPosition);
    }

    // =========================================================
    //  HELPERS
    // =========================================================

    private void setServoPosition(double pos) {
        pos = clamp(pos);
        double pos2 = ServoConfig.servo2Mirrored ? (1.0 - pos) : pos;
        spindexerServo1.setPosition(pos);
        spindexerServo2.setPosition(pos2);
    }

    private double clamp(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }
}

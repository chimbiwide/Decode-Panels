package org.firstinspires.ftc.teamcode.testTeleOp;

import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tools.BallDetector;
import org.firstinspires.ftc.teamcode.tools.Color;
import org.firstinspires.ftc.teamcode.tools.Drive;
import org.firstinspires.ftc.teamcode.tools.Flywheel;
import org.firstinspires.ftc.teamcode.tools.Intake;
import org.firstinspires.ftc.teamcode.tools.RGBDisplay;
import org.firstinspires.ftc.teamcode.tools.Sorter;
import org.firstinspires.ftc.teamcode.tools.Turret;

@TeleOp(name = "Blue Tele")
public class Blue extends LinearOpMode {

    TelemetryManager.TelemetryWrapper panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    private enum IntakeMode { NORMAL, SORTING }

    private Limelight3A limelight;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    private IntakeMode intakeMode = IntakeMode.NORMAL;
    private boolean shootingMode = false;
    private boolean sensorsEnabled = false;
    private boolean sensorsJustEnabled = false;

    private boolean ballDetectedLastLoop = false;
    private boolean ballDetectPending = false;
    private double ballDetectPendingStart = 0;
    private boolean awaitingColorDetect = false;
    private double colorDetectStartTime = 0;
    private boolean manualSpinUsed = false;
    private boolean triggerSpindexerEnabled = false;
    private boolean patternLocked = false;
    private double pendingSettleDelay = 0;

    private enum JamState { IDLE, OUTTAKING }
    private JamState jamState = JamState.IDLE;
    private double jamActionStart = 0;
    private boolean jamIntakeWasOn = false;
    private boolean equationOverride = false;

    private boolean lastCrossG1, lastCrossG2;
    private boolean lastTriangleG1, lastTriangleG2;
    private boolean lastCircleG1, lastCircleG2;
    private boolean lastSquareG1, lastSquareG2;
    private boolean lastDpadUpG1, lastDpadUpG2;
    private boolean lastDpadDownG1, lastDpadDownG2;
    private boolean lastDpadLeftG1, lastDpadLeftG2;
    private boolean lastDpadRightG2;
    private boolean lastShareG1, lastShareG2;
    private boolean lastOptionsG1, lastOptionsG2;

    @Override
    public void runOpMode() {
        Drive.init(hardwareMap);
        Flywheel.init(hardwareMap);
        Turret.init(hardwareMap);
        Sorter.init(hardwareMap);
        Color.init(hardwareMap);
        Intake.init(hardwareMap);
        RGBDisplay.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

        BallDetector.init(hardwareMap,
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "colorSensor1"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "colorSensor2"));

        panelsTelemetry.update();
        waitForStart();
        timer.reset();
        lastTime = timer.seconds();

        while (opModeIsActive()) {
            double currentTime = timer.seconds();
            double dt = Math.max(currentTime - lastTime, 1e-3);
            lastTime = currentTime;
            double nowMs = timer.milliseconds();

            Drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            Sorter.setSortingMode(intakeMode == IntakeMode.SORTING);
            Sorter.updateSlew();
            Sorter.checkPendingTransfer();
            Flywheel.update(dt);

            LLResult result = limelight.getLatestResult();
            Double tx = (result != null && result.isValid()) ? result.getTx() : null;
            if (equationOverride) {
                boolean atPreset = Math.abs(Flywheel.getVelocity() - Flywheel.FlywheelPID.targetVelocity) <= Flywheel.FlywheelPID.blinkTolerance;
                if (atPreset && result != null && result.isValid()) equationOverride = false;
            }
            if (Flywheel.FlywheelEquation.enableEquation && !equationOverride && result != null && result.isValid()) {
                Flywheel.FlywheelPID.targetVelocity = Flywheel.calculateFromTy(result.getTy());
            }
            boolean lb = gamepad1.left_bumper || gamepad2.left_bumper;
            boolean rb = gamepad1.right_bumper || gamepad2.right_bumper;
            Turret.update(dt, tx, lb, rb);

            double g1ManualServo = 0;
            if (gamepad1.right_trigger > 0.05) g1ManualServo += gamepad1.right_trigger * Sorter.SpindexerDelays.manualTriggerSpeed;
            if (gamepad1.left_trigger > 0.05) g1ManualServo -= gamepad1.left_trigger * Sorter.SpindexerDelays.manualTriggerSpeed;
            if (g1ManualServo != 0) {
                Sorter.manualRotate(g1ManualServo);
                manualSpinUsed = true;
            }
            Sorter.updateTransferMotor(0);

            if (triggerSpindexerEnabled) {
                double g2ManualServo = 0;
                if (gamepad2.right_trigger > 0.05) g2ManualServo += gamepad2.right_trigger * 0.005;
                if (gamepad2.left_trigger > 0.05) g2ManualServo -= gamepad2.left_trigger * 0.005;
                if (g2ManualServo != 0) {
                    Sorter.manualRotate(g2ManualServo);
                    manualSpinUsed = true;
                }
            }

            handleTransfer();

            boolean wasSensorsEnabled = sensorsEnabled;
            if (!sensorsEnabled
                    && timer.milliseconds() >= BallDetector.BallDetection.startupDelayMs
                    && Sorter.isSettled()) {
                sensorsEnabled = true;
                Sorter.clearSensorGate();
            }
            if (Sorter.isSensorsGated()) sensorsEnabled = false;
            sensorsJustEnabled = sensorsEnabled && !wasSensorsEnabled;
            Color.setLEDs(sensorsEnabled && Color.ColorConfig.enableLEDs);

            handleJam(nowMs);
            handleBallDetection(nowMs);
            handleIntakeControls(nowMs);
            handleShootingControls();
            handlePatternControls();
            RGBDisplay.update(intakeMode == IntakeMode.SORTING);

            if (Flywheel.checkRumble(50)) {
                gamepad1.rumble(1.0, 1.0, 1000);
            }

            updateTelemetry(result);
        }

        Flywheel.stop();
        Turret.stop();
        Intake.stop();
        Sorter.stopAll();
        Drive.stop();
        limelight.stop();
    }

    private void handleJam(double nowMs) {
        if (jamState == JamState.IDLE) {
            if (Sorter.isJammed()) {
                jamState = JamState.OUTTAKING;
                jamActionStart = nowMs;
                jamIntakeWasOn = Intake.isToggled();
                Intake.setToggled(false);
                ballDetectPending = false;
                awaitingColorDetect = false;
            }
        } else if (jamState == JamState.OUTTAKING) {
            if ((nowMs - jamActionStart) >= 1000) {
                Sorter.removeBall();
                manualSpinUsed = true;
                if (jamIntakeWasOn) Intake.setToggled(true);
                jamState = JamState.IDLE;
            }
        }
    }

    private void handleBallDetection(double nowMs) {
        boolean ballNow = sensorsEnabled && BallDetector.detectBall();

        if (sensorsJustEnabled && ballNow) {
            ballDetectedLastLoop = true;
            return;
        }

        boolean ballAdded = false;

        if (sensorsEnabled && Intake.isToggled() && !Sorter.isTransferActive()
                && !Sorter.isTransferPending()
                && !manualSpinUsed && !Intake.isPaused() && !ballDetectPending && !awaitingColorDetect) {
            if (ballNow && !ballDetectedLastLoop && Sorter.getBallCount() < 3) {
                ballDetectPending = true;
                ballDetectPendingStart = nowMs;
                pendingSettleDelay = BallDetector.isColorDistanceTriggered() ? 0 : BallDetector.BallDetection.analogSettleDelayMs;
            }
        }

        if (ballDetectPending && !awaitingColorDetect && Sorter.getBallCount() < 3) {
            if ((nowMs - ballDetectPendingStart) >= pendingSettleDelay) {
                ballDetectPending = false;
                if (intakeMode == IntakeMode.NORMAL) {
                    Sorter.commitBall("unknown");
                    ballAdded = true;
                } else {
                    awaitingColorDetect = true;
                    colorDetectStartTime = nowMs;
                }
            }
        }

        if (awaitingColorDetect && Sorter.getBallCount() < 3) {
            String color = Color.detectBallColor();
            boolean timedOut = (nowMs - colorDetectStartTime) > Color.ColorConfig.colorTimeoutMs;
            if (color != null || timedOut) {
                String committed = (color != null) ? color : "purple";
                Color.updateLastDetected(committed);
                Sorter.commitBall(committed);
                awaitingColorDetect = false;
                ballAdded = true;
            }
        }

        if (sensorsEnabled && intakeMode == IntakeMode.SORTING) {
            String c = Color.detectBallColor();
            Color.updateLastDetected(c);
        }

        if (ballAdded) {
            Intake.triggerPause(nowMs);
            if (Sorter.getBallCount() == 3) {
                Intake.setToggled(false);
                gamepad1.rumble(0.5, 0.5, 300);
                gamepad2.rumble(0.5, 0.5, 300);
            }
        }
        if (manualSpinUsed) {
            manualSpinUsed = false;
            ballDetectedLastLoop = false;
        }
        ballDetectedLastLoop = ballNow;
    }

    private void handleIntakeControls(double nowMs) {
        boolean g1Cross = gamepad1.cross;
        if (g1Cross && !lastCrossG1) Intake.toggle();
        lastCrossG1 = g1Cross;

        boolean g2Cross = gamepad2.cross;
        if (g2Cross && !lastCrossG2) Intake.toggle();
        lastCrossG2 = g2Cross;

        Intake.update(nowMs, gamepad1.dpad_right || jamState == JamState.OUTTAKING);

        boolean g1Sq = gamepad1.square;
        if (g1Sq && !lastSquareG1 && !Sorter.isTransferActive()) {
            Sorter.manualAddBall();
            manualSpinUsed = true;
            if (Sorter.getBallCount() == 3) gamepad1.rumble(0.5, 0.5, 200);
        }
        lastSquareG1 = g1Sq;

        boolean g2Sq = gamepad2.square;
        if (g2Sq && !lastSquareG2 && !Sorter.isTransferActive()) {
            Sorter.manualAddBall();
            manualSpinUsed = true;
            if (Sorter.getBallCount() == 3) gamepad2.rumble(0.5, 0.5, 200);
        }
        lastSquareG2 = g2Sq;

        boolean g1DpadLeft = gamepad1.dpad_left;
        if (g1DpadLeft && !lastDpadLeftG1 && !Sorter.isTransferActive()) {
            Sorter.reverseSpindexer();
            manualSpinUsed = true;
        }
        lastDpadLeftG1 = g1DpadLeft;

        boolean g1DpadDown = gamepad1.dpad_down;
        if (g1DpadDown && !lastDpadDownG1) {
            Sorter.zeroServos();
            Intake.setToggled(false);
            shootingMode = false;
            awaitingColorDetect = false;
            ballDetectPending = false;
        }
        lastDpadDownG1 = g1DpadDown;
    }

    private void handleShootingControls() {
        boolean g1Tri = gamepad1.triangle;
        if (g1Tri && !lastTriangleG1 && !Sorter.isTransferActive()) {
            shootingMode = true;
            Sorter.rotateForShooting(intakeMode == IntakeMode.SORTING);
        }
        lastTriangleG1 = g1Tri;

        boolean g2Tri = gamepad2.triangle;
        if (g2Tri && !lastTriangleG2) {
            Turret.toggleTracking();
            if (Turret.isTrackingEnabled()) {
                gamepad2.rumble(0.8, 0.8, 300);
            } else {
                gamepad2.rumble(0.3, 0.3, 150);
            }
        }
        lastTriangleG2 = g2Tri;

        boolean g1Circle = gamepad1.circle;
        if (g1Circle && !lastCircleG1 && !Sorter.isTransferActive() && !Sorter.isTransferPending()) {
            if (!Sorter.startTransfer()) {
                gamepad1.rumble(0.5, 0.5, 200);
            }
        }
        lastCircleG1 = g1Circle;

        boolean g1DpadUp = gamepad1.dpad_up;
        if (g1DpadUp && !lastDpadUpG1) {
            Turret.toggleTracking();
            if (Turret.isTrackingEnabled()) {
                gamepad1.rumble(0.8, 0.8, 300);
            } else {
                gamepad1.rumble(0.3, 0.3, 150);
            }
        }
        lastDpadUpG1 = g1DpadUp;

        boolean g2Circle = gamepad2.circle;
        if (g2Circle && !lastCircleG2) {
            triggerSpindexerEnabled = !triggerSpindexerEnabled;
            gamepad2.rumble(triggerSpindexerEnabled ? 0.8 : 0.3, triggerSpindexerEnabled ? 0.8 : 0.3, 200);
        }
        lastCircleG2 = g2Circle;

        boolean g1Share = gamepad1.share;
        if (g1Share && !lastShareG1) {
            Flywheel.FlywheelPID.targetVelocity = Flywheel.FlywheelPID.closeRangeVelocity;
            Flywheel.resetRumble();
            equationOverride = true;
        }
        lastShareG1 = g1Share;

        boolean g1Options = gamepad1.options;
        if (g1Options && !lastOptionsG1) {
            Flywheel.FlywheelPID.targetVelocity = Flywheel.FlywheelPID.longRangeVelocity;
            Flywheel.resetRumble();
            equationOverride = true;
        }
        lastOptionsG1 = g1Options;

        boolean g2Share = gamepad2.share;
        if (g2Share && !lastShareG2) {
            Flywheel.FlywheelPID.targetVelocity = Flywheel.FlywheelPID.closeRangeVelocity;
            Flywheel.resetRumble();
            equationOverride = true;
        }
        lastShareG2 = g2Share;

        boolean g2Options = gamepad2.options;
        if (g2Options && !lastOptionsG2) {
            Flywheel.FlywheelPID.targetVelocity = Flywheel.FlywheelPID.longRangeVelocity;
            Flywheel.resetRumble();
            equationOverride = true;
        }
        lastOptionsG2 = g2Options;

    }

    private void handlePatternControls() {
        boolean g2DpadUp = gamepad2.dpad_up;
        if (g2DpadUp && !lastDpadUpG2) {
            intakeMode = (intakeMode == IntakeMode.NORMAL) ? IntakeMode.SORTING : IntakeMode.NORMAL;
            awaitingColorDetect = false;
            ballDetectPending = false;
            boolean sorting = intakeMode == IntakeMode.SORTING;
            gamepad1.rumble(sorting ? 0.8 : 0.3, sorting ? 0.8 : 0.3, 200);
            gamepad2.rumble(sorting ? 0.8 : 0.3, sorting ? 0.8 : 0.3, 200);
        }
        lastDpadUpG2 = g2DpadUp;

        boolean g2Left = gamepad2.dpad_left;
        if (g2Left && !lastDpadLeftG2 && !patternLocked) {
            Sorter.setPattern(new String[]{"purple", "purple", "green"});
            Sorter.ShootingPattern.enablePatternMatching = true;
            intakeMode = IntakeMode.SORTING;
            patternLocked = true;
            gamepad2.rumble(0.8, 0.8, 400);
        }
        lastDpadLeftG2 = g2Left;

        boolean g2Down = gamepad2.dpad_down;
        if (g2Down && !lastDpadDownG2 && !patternLocked) {
            Sorter.setPattern(new String[]{"purple", "green", "purple"});
            Sorter.ShootingPattern.enablePatternMatching = true;
            intakeMode = IntakeMode.SORTING;
            patternLocked = true;
            gamepad2.rumble(0.8, 0.8, 400);
        }
        lastDpadDownG2 = g2Down;

        boolean g2Right = gamepad2.dpad_right;
        if (g2Right && !lastDpadRightG2 && !patternLocked) {
            Sorter.setPattern(new String[]{"green", "purple", "purple"});
            Sorter.ShootingPattern.enablePatternMatching = true;
            intakeMode = IntakeMode.SORTING;
            patternLocked = true;
            gamepad2.rumble(0.8, 0.8, 400);
        }
        lastDpadRightG2 = g2Right;
    }

    private void handleTransfer() {
        int result = Sorter.handleTransfer();
        if (result != 0 && result != -1) {
            shootingMode = false;
            if (Sorter.TransferSettings.autoPauseAfterShoot) {
                Intake.setToggled(false);
            }
            if (result == 2) {
                gamepad1.rumble(1.0, 1.0, 300);
            }
        }
    }

    private void updateTelemetry(LLResult result) {
        boolean valid = result != null && result.isValid();

        Color.readSensors();
        double[] cs1 = Color.getCS1();
        double[] cs2 = Color.getCS2();

        telemetry.addData("Intake", Intake.isToggled() ? "ON" : "OFF");
        telemetry.addData("Mode", intakeMode.name() + (Sorter.ShootingPattern.enablePatternMatching ? " [PATTERN]" : ""));
        telemetry.addData("Balls", Sorter.getBallCount() + "/3  " + Sorter.formatBallSlots());
        telemetry.addData("Shoot", shootingMode ? "READY -> " + Sorter.formatShootingOrder() : "COLLECT");
        telemetry.addData("Transfer", Sorter.isTransferActive() ? "SHOOTING" : Sorter.isTransferPending() ? "RESETTING" : "IDLE");
        telemetry.addData("Flywheel", String.format("%.0f / %.0f  eq=%s", Flywheel.getVelocity(), Flywheel.FlywheelPID.targetVelocity,
                Flywheel.FlywheelEquation.enableEquation ? "ON" : "OFF"));
        telemetry.addData("Tracking", Turret.isTrackingEnabled() ? "ON" : "OFF");
        telemetry.addData("Turret", Turret.getPosition() + " ticks");
        telemetry.addData("Spindexer", (Sorter.getStep() == -1 ? "INIT" : "Slot " + (Sorter.getStep() + 1))
                + " pos=" + String.format("%.3f", Sorter.getCurrentPos()));
        telemetry.addData("Ranger", String.format("%.2f in", BallDetector.getRangerDistanceIn()));
        telemetry.addData("CS1 Dist", BallDetector.hasCS1Distance() ? String.format("%.1f mm", BallDetector.getCS1DistanceMm()) : "N/A");
        telemetry.addData("CS2 Dist", BallDetector.hasCS2Distance() ? String.format("%.1f mm", BallDetector.getCS2DistanceMm()) : "N/A");
        telemetry.addData("CS1", String.format("R%.3f G%.3f B%.3f %s", cs1[0], cs1[1], cs1[2],
                Color.isCS1Green() ? "GREEN" : Color.isCS1Purple() ? "PURPLE" : ""));
        telemetry.addData("CS2", String.format("R%.3f G%.3f B%.3f %s", cs2[0], cs2[1], cs2[2],
                Color.isCS2Green() ? "GREEN" : Color.isCS2Purple() ? "PURPLE" : ""));
        telemetry.addData("Color", Color.getLastDetected() != null ? Color.getLastDetected().toUpperCase() : "NONE");
        telemetry.update();

        panelsTelemetry.addData("=== COLOR SENSOR ONE ===", Color.ColorConfig.enableSensor1 ? "ENABLED" : "DISABLED");
        panelsTelemetry.addData("CS1 Gain", Color.ColorSensor1.gain);
        panelsTelemetry.addData("CS1 Alpha", String.format("%.4f", cs1[3]));
        panelsTelemetry.addData("CS1 R / G / B", String.format("%.4f  /  %.4f  /  %.4f", cs1[0], cs1[1], cs1[2]));
        panelsTelemetry.addData("CS1 Green", String.format("%.4f  thresh=%.3f  %s",
                cs1[1], Color.ColorThresholds.greenThreshold, cs1[1] >= Color.ColorThresholds.greenThreshold ? "-> GREEN" : ""));
        panelsTelemetry.addData("CS1 Blue", String.format("%.4f  thresh=%.3f  %s",
                cs1[2], Color.ColorThresholds.purpleThreshold, cs1[2] >= Color.ColorThresholds.purpleThreshold ? "-> PURPLE" : ""));
        panelsTelemetry.addData("CS1 Result", Color.isCS1Green() ? "GREEN" : Color.isCS1Purple() ? "PURPLE" : "none");
        panelsTelemetry.addData("", "");

        panelsTelemetry.addData("=== COLOR SENSOR TWO ===", Color.ColorConfig.enableSensor2 ? "ENABLED" : "DISABLED");
        panelsTelemetry.addData("CS2 Gain", Color.ColorSensor2.gain);
        panelsTelemetry.addData("CS2 Alpha", String.format("%.4f", cs2[3]));
        panelsTelemetry.addData("CS2 R / G / B", String.format("%.4f  /  %.4f  /  %.4f", cs2[0], cs2[1], cs2[2]));
        panelsTelemetry.addData("CS2 Green", String.format("%.4f  thresh=%.3f  %s",
                cs2[1], Color.ColorThresholds.greenThreshold, cs2[1] >= Color.ColorThresholds.greenThreshold ? "-> GREEN" : ""));
        panelsTelemetry.addData("CS2 Blue", String.format("%.4f  thresh=%.3f  %s",
                cs2[2], Color.ColorThresholds.purpleThreshold, cs2[2] >= Color.ColorThresholds.purpleThreshold ? "-> PURPLE" : ""));
        panelsTelemetry.addData("CS2 Result", Color.isCS2Green() ? "GREEN" : Color.isCS2Purple() ? "PURPLE" : "none");
        panelsTelemetry.addData("Last Color Detected", Color.getLastDetected() != null ? Color.getLastDetected().toUpperCase() : "NONE");
        panelsTelemetry.addData("Color Timeout", Color.ColorConfig.colorTimeoutMs + "ms");
        panelsTelemetry.addData("", "");

        panelsTelemetry.addData("=== SERVO SPEED ===", "");
        panelsTelemetry.addData("Intake Speed", Sorter.SpindexerDelays.intakeServoSpeed);
        panelsTelemetry.addData("Sorting Speed", Sorter.SpindexerDelays.sortingServoSpeed);
        panelsTelemetry.addData("Shoot Hi-Vel Speed (>=" + (int) Sorter.SpindexerDelays.shootVelocityThreshold + ")", Sorter.SpindexerDelays.shootHighVelSpeed);
        panelsTelemetry.addData("Shoot Lo-Vel Speed (<" + (int) Sorter.SpindexerDelays.shootVelocityThreshold + ")", Sorter.SpindexerDelays.shootLowVelSpeed);
        panelsTelemetry.addData("Active", String.format("%.2f (%s)",
                Sorter.getActiveServoSpeed(), Sorter.isTransferActive() ? "SHOOTING" : "INTAKE"));
        panelsTelemetry.addData("", "");

        panelsTelemetry.addData("=== INTAKE ===", "");
        panelsTelemetry.addData("Intake", Intake.isToggled() ? "ON" : "OFF");
        panelsTelemetry.addData("Mode", intakeMode.name());
        panelsTelemetry.addData("Balls", Sorter.getBallCount() + "/3  " + Sorter.formatBallSlots());
        panelsTelemetry.addData("Shoot Order", Sorter.formatShootingOrder());
        panelsTelemetry.addData("Pattern", String.join(" > ", Sorter.getCurrentPattern()) + (patternLocked ? "  [LOCKED]" : "  [unlocked]"));
        panelsTelemetry.addData("Pattern Match", Sorter.ShootingPattern.enablePatternMatching ? "ON" : "OFF");
        panelsTelemetry.addData("", "");

        panelsTelemetry.addData("=== SPINDEXER ===", "");
        panelsTelemetry.addData("Trigger Spindexer", triggerSpindexerEnabled ? "ON (G2 triggers = spin)" : "OFF (G2 circle to enable)");
        panelsTelemetry.addData("Slot", Sorter.getStep() == -1 ? "INIT" : "Slot " + (Sorter.getStep() + 1));
        panelsTelemetry.addData("Servo1 pos", String.format("%.4f", Sorter.getCurrentPos()));
        panelsTelemetry.addData("Servo2 pos", String.format("%.4f", 1.0 - Sorter.getCurrentPos()));
        panelsTelemetry.addData("Settle", Sorter.isSettled() ? "SETTLED" : "MOVING");
        panelsTelemetry.addData("Transfer", Sorter.isTransferActive()
                ? "ACTIVE " + (int) Sorter.getTransferTimeLeft() + "ms left"
                : Sorter.isTransferPending() ? "RESETTING TO SLOT1" : "IDLE");
        panelsTelemetry.addData("", "");

        panelsTelemetry.addData("=== DISTANCE SENSORS ===", "");
        panelsTelemetry.addData("Sensor Gate", sensorsEnabled ? "ON" : "WAITING (settling)");
        panelsTelemetry.addData("Analog Ranger", BallDetector.BallDetection.useAnalogRanger ? "ENABLED" : "DISABLED");
        panelsTelemetry.addData("Ranger Distance (in)", String.format("%.3f  thresh=%.2f  %s",
                BallDetector.getRangerDistanceIn(), BallDetector.BallDetection.analogThresholdIn,
                BallDetector.getRangerDistanceIn() <= BallDetector.BallDetection.analogThresholdIn ? "BALL" : "clear"));
        panelsTelemetry.addData("Ranger Voltage", String.format("%.4fV", BallDetector.getRangerVoltage()));
        panelsTelemetry.addData("Color Sensor Distance", BallDetector.BallDetection.useColorSensorDistance ? "ENABLED" : "DISABLED");
        if (BallDetector.BallDetection.useColorSensorDistance) {
            if (BallDetector.hasCS1Distance()) panelsTelemetry.addData("CS1 Distance (mm)", String.format("%.1f  thresh=%.1f  %s",
                    BallDetector.getCS1DistanceMm(), BallDetector.BallDetection.csThresholdMm,
                    BallDetector.getCS1DistanceMm() <= BallDetector.BallDetection.csThresholdMm ? "BALL" : "clear"));
            if (BallDetector.hasCS2Distance()) panelsTelemetry.addData("CS2 Distance (mm)", String.format("%.1f  thresh=%.1f  %s",
                    BallDetector.getCS2DistanceMm(), BallDetector.BallDetection.csThresholdMm,
                    BallDetector.getCS2DistanceMm() <= BallDetector.BallDetection.csThresholdMm ? "BALL" : "clear"));
        }
        panelsTelemetry.addData("Color Sensors", BallDetector.BallDetection.useColorSensors ? "ENABLED" : "DISABLED");
        panelsTelemetry.addData("Ball Detected", (sensorsEnabled && BallDetector.detectBall()) ? "YES" : "NO");
        panelsTelemetry.addData("Analog Settle Delay", BallDetector.BallDetection.analogSettleDelayMs + "ms  (color dist sensor = 0ms)");
        panelsTelemetry.addData("Startup Delay", BallDetector.BallDetection.startupDelayMs + "ms");
        panelsTelemetry.addData("", "");

        panelsTelemetry.addData("=== FLYWHEEL ===", "");
        panelsTelemetry.addData("Equation", Flywheel.FlywheelEquation.enableEquation ? "ON (G2 dpad_right to off)" : "OFF (G2 dpad_right to on)");
        panelsTelemetry.addData("Eq tyA", Flywheel.FlywheelEquation.tyA);
        panelsTelemetry.addData("Eq tyB", Flywheel.FlywheelEquation.tyB);
        panelsTelemetry.addData("Target", String.format("%.1f tks/s", Flywheel.FlywheelPID.targetVelocity));
        panelsTelemetry.addData("Actual", String.format("%.1f tks/s", Flywheel.getVelocity()));
        panelsTelemetry.addData("Error", String.format("%.1f", Flywheel.getError()));
        panelsTelemetry.addData("Power", String.format("%.3f", Flywheel.getPower()));
        panelsTelemetry.addData("Close Preset", Flywheel.FlywheelPID.closeRangeVelocity);
        panelsTelemetry.addData("Long Preset", Flywheel.FlywheelPID.longRangeVelocity);
        panelsTelemetry.addData("Transfer Motor", String.format("%.3f", Sorter.getTransferMotorPower()));
        panelsTelemetry.addData("", "");

        panelsTelemetry.addData("=== TURRET ===", "");
        if (Turret.isWraparoundActive()) {
            panelsTelemetry.addData("Turret", "WRAPPING -> " + Turret.getWraparoundTarget());
        } else {
            panelsTelemetry.addData("Turret Ticks", Turret.getPosition());
            panelsTelemetry.addData("Turret Power", String.format("%.3f", Turret.getPower()));
        }
        panelsTelemetry.addData("Tracking", Turret.isTrackingEnabled() ? "ON" : "OFF");
        panelsTelemetry.addData("Deadzone", String.format("+/- %.2f", Turret.TurretTracking.txDeadzone));
        if (valid) {
            double txVal = result.getTx();
            panelsTelemetry.addData("tx", String.format("%.3f  %s", txVal,
                    Math.abs(txVal) <= Turret.TurretTracking.txDeadzone ? "LOCKED" : "TRACKING"));
        } else {
            panelsTelemetry.addData("Limelight", "NO TARGET");
        }
        panelsTelemetry.update();
    }
}

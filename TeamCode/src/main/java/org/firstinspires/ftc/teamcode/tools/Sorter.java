package org.firstinspires.ftc.teamcode.tools;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Sorter {

    @Configurable
    public static class SpindexerPositions {
        public static double initPosition = 0.5;
        public static double slot1 = 0.097;
        public static double slot2 = 0.2854;
        public static double slot3 = 0.4737;
        public static double fullRotation = 0.5653;
        public static double shootTolerance = 0.05;
    }

    @Configurable
    public static class SpindexerDelays {
        public static double transferDurationMs = 1500;
        public static double settleTimeMs = 200;
        public static double intakeServoSpeed = 0.5;
        public static double sortingServoSpeed = 0.5;
        public static double shootHighVelSpeed = 0.5;
        public static double shootLowVelSpeed = 0.9;
        public static double shootVelocityThreshold = 1375.0;
        public static double pauseOnDetectMs = 300;
        public static double jamTimeoutMs = 1500;
        public static double jamTolerance = 0.05;
        public static double manualTriggerSpeed = 0.003;
    }

    @Configurable
    public static class TransferSettings {
        public static boolean preventZeroBallTransfer = false;
        public static double transferTimeoutMs = 5000;
        public static boolean enableTransferTimeout = true;
        public static boolean autoPauseAfterShoot = true;
        public static double shootPower = -1.0;
        public static double manualPower = 1.0;
    }

    @Configurable
    public static class ShootingPattern {
        public static String pattern = "green,purple,purple";
        public static boolean enablePatternMatching = true;
        public static boolean enablePatternCycling = true;
    }

    private static Servo servo1;
    private static Servo servo2;
    private static DcMotorEx transferMotor;
    private static final ElapsedTime timer = new ElapsedTime();
    private static final ElapsedTime slewTimer = new ElapsedTime();

    private static int currentPort = 0;
    private static double targetPos = 0.5;
    private static double currentPos = 0.5;
    private static double moveTime = 0;
    private static boolean sensorsGated = true;

    private static boolean sortingModeActive = false;
    private static boolean transferActive = false;
    private static boolean transferPending = false;
    private static double transferStartTime = 0;
    private static int originalBallCount = 0;

    private static String[] ports = new String[3];
    private static String[] currentPattern = {"green", "purple", "purple"};

    public static void init(HardwareMap map) {
        servo1 = map.get(Servo.class, "servo1");
        servo2 = map.get(Servo.class, "servo2");
        transferMotor = map.get(DcMotorEx.class, "transfer");
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        currentPort = 0;
        targetPos = SpindexerPositions.initPosition;
        currentPos = SpindexerPositions.initPosition;
        moveTime = 0;
        sensorsGated = true;
        transferActive = false;
        transferPending = false;
        ports = new String[3];

        String[] initial = ShootingPattern.pattern.split(",");
        if (initial.length == 3) {
            currentPattern = new String[]{initial[0].trim(), initial[1].trim(), initial[2].trim()};
        } else {
            currentPattern = new String[]{"green", "purple", "purple"};
        }

        servo1.setPosition(currentPos);
        servo2.setPosition(1.0 - currentPos);
        timer.reset();
        slewTimer.reset();
    }

    public static void setPosition(double pos) {
        targetPos = Math.max(0.0, Math.min(1.0, pos));
        moveTime = timer.milliseconds();
        sensorsGated = true;
    }

    public static void updateSlew() {
        double elapsed = slewTimer.seconds();
        slewTimer.reset();
        double speed = sortingModeActive ? SpindexerDelays.sortingServoSpeed
                : transferActive
                    ? (Flywheel.getVelocity() >= SpindexerDelays.shootVelocityThreshold
                            ? SpindexerDelays.shootHighVelSpeed
                            : SpindexerDelays.shootLowVelSpeed)
                    : SpindexerDelays.intakeServoSpeed;
        double maxStep = speed * elapsed;
        double error = targetPos - currentPos;
        if (Math.abs(error) <= maxStep) {
            currentPos = targetPos;
        } else {
            currentPos += Math.signum(error) * maxStep;
        }
        servo1.setPosition(currentPos);
        servo2.setPosition(1.0 - currentPos);
    }

    public static boolean isSettled() {
        return (timer.milliseconds() - moveTime) >= SpindexerDelays.settleTimeMs
                && Math.abs(currentPos - targetPos) <= 0.02;
    }

    public static boolean isJammed() {
        if (transferActive || transferPending) return false;
        if (moveTime == 0) return false;
        double elapsed = timer.milliseconds() - moveTime;
        return elapsed > SpindexerDelays.jamTimeoutMs
                && Math.abs(currentPos - targetPos) > SpindexerDelays.jamTolerance;
    }

    public static boolean isSensorsGated() { return sensorsGated; }
    public static void clearSensorGate() { sensorsGated = false; }

    public static double getSlotPosition(int slot) {
        switch (slot) {
            case 0: return SpindexerPositions.slot1;
            case 1: return SpindexerPositions.slot2;
            case 2: return SpindexerPositions.slot3;
            default: return SpindexerPositions.initPosition;
        }
    }

    private static void shufflePorts(int turns) {
        String[] newPorts = new String[3];
        for (int i = 0; i < 3; i++) {
            int index = ((i - turns) % 3 + 3) % 3;
            newPorts[i] = ports[index];
        }
        ports = newPorts;
    }

    public static void turnToPort(int port) {
        int turns = port - currentPort;
        shufflePorts(turns);
        currentPort = port;
        setPosition(getSlotPosition(port));
    }

    public static void advanceSpindexer() {
        if (currentPort < 2) {
            currentPort++;
            shufflePorts(1);
            setPosition(getSlotPosition(currentPort));
        }
    }

    public static void reverseSpindexer() {
        if (currentPort > 0) {
            currentPort--;
            shufflePorts(-1);
            setPosition(getSlotPosition(currentPort));
        } else {
            currentPort = 0;
            setPosition(SpindexerPositions.initPosition);
        }
    }

    public static void activateSlot1() {
        if (currentPort == 0 && Math.abs(currentPos - SpindexerPositions.slot1) > 0.02) {
            setPosition(SpindexerPositions.slot1);
        }
    }

    public static void commitBall(String color) {
        ports[0] = color;
        advanceSpindexer();
    }

    public static void removeBall() {
        if (getBallCount() > 0) {
            ports[0] = null;
            reverseSpindexer();
        }
    }

    public static void manualAddBall() {
        if (getBallCount() < 3) {
            ports[0] = "unknown";
            advanceSpindexer();
        } else {
            advanceSpindexer();
        }
    }

    public static void clearBalls() {
        ports = new String[3];
        currentPort = 0;
        setPosition(SpindexerPositions.slot1);
    }

    public static void zeroServos() {
        currentPort = 0;
        targetPos = 0.0;
        currentPos = 0.0;
        servo1.setPosition(0.0);
        servo2.setPosition(1.0);
        sensorsGated = true;
        transferActive = false;
        transferPending = false;
        ports = new String[3];
    }

    public static int getBallCount() {
        int count = 0;
        for (String p : ports) {
            if (p != null) count++;
        }
        return count;
    }

    public static boolean isFull() {
        for (String p : ports) {
            if (p == null) return false;
        }
        return true;
    }

    public static boolean readyForPattern() {
        int green = 0, purple = 0;
        for (String p : ports) {
            if (p != null) {
                if (p.equals("green")) purple++;
                else green++;
            }
        }
        return green == 1 && purple == 2;
    }

    private static int getGreenBall() {
        for (int i = 0; i < ports.length; i++) {
            if (ports[i] != null && ports[i].equals("green")) return i;
        }
        return -1;
    }

    public static void rotateForShooting(boolean sorting) {
        if (!sorting || !ShootingPattern.enablePatternMatching || getBallCount() == 0) {
            turnToPort(0);
            return;
        }
        if (!readyForPattern()) {
            turnToPort(0);
            return;
        }
        int greenIdx = getGreenBall();
        if (greenIdx == -1) {
            turnToPort(0);
            return;
        }
        String first = currentPattern[0].trim();
        String second = currentPattern[1].trim();
        String third = currentPattern[2].trim();

        if (first.equals("green")) {
            if (greenIdx == 0) turnToPort(0);
            else if (greenIdx == 2) turnToPort(1);
            else turnToPort(0);
        } else if (second.equals("green")) {
            if (greenIdx == 2) turnToPort(0);
            else if (greenIdx == 1) turnToPort(1);
            else turnToPort(0);
        } else if (third.equals("green")) {
            if (greenIdx == 1) turnToPort(0);
            else if (greenIdx == 0) turnToPort(1);
            else turnToPort(0);
        } else {
            turnToPort(0);
        }
    }

    public static boolean startTransfer() {
        if (TransferSettings.preventZeroBallTransfer && getBallCount() == 0) return false;
        double maxAllowed = 1.0 - SpindexerPositions.fullRotation - SpindexerPositions.shootTolerance;
        if (currentPos > maxAllowed) {
            turnToPort(0);
            transferPending = true;
            return true;
        }
        beginTransferSpin();
        return true;
    }

    private static void beginTransferSpin() {
        double startPos = currentPos;
        setPosition(startPos + SpindexerPositions.fullRotation);
        originalBallCount = getBallCount();
        transferActive = true;
        transferPending = false;
        transferStartTime = timer.milliseconds();
    }

    public static void checkPendingTransfer() {
        if (transferPending && isSettled()) {
            beginTransferSpin();
        }
    }

    public static void manualRotate(double delta) {
        if (transferActive || transferPending) return;
        currentPos = Math.max(0.0, Math.min(1.0, currentPos + delta));
        targetPos = currentPos;
        servo1.setPosition(currentPos);
        servo2.setPosition(1.0 - currentPos);
    }

    public static boolean isTransferPending() { return transferPending; }

    public static int handleTransfer() {
        if (!transferActive) return 0;
        double elapsed = timer.milliseconds() - transferStartTime;
        if (TransferSettings.enableTransferTimeout && elapsed > TransferSettings.transferTimeoutMs) {
            transferActive = false;
            clearBalls();
            return 2;
        }
        if (elapsed >= SpindexerDelays.transferDurationMs) {
            transferActive = false;
            if (ShootingPattern.enablePatternCycling && originalBallCount == 3) {
                String t = currentPattern[0];
                currentPattern[0] = currentPattern[1];
                currentPattern[1] = currentPattern[2];
                currentPattern[2] = t;
            }
            clearBalls();
            return 1;
        }
        return -1;
    }

    public static void updateTransferMotor(double manualPower) {
        if (transferActive) {
            transferMotor.setPower(TransferSettings.shootPower);
        } else {
            transferMotor.setPower(Math.max(-1.0, Math.min(1.0, manualPower)) * TransferSettings.manualPower);
        }
    }

    public static void setPattern(String[] p) {
        if (p.length == 3) {
            currentPattern[0] = p[0];
            currentPattern[1] = p[1];
            currentPattern[2] = p[2];
        }
    }

    public static void setSortingMode(boolean sorting) { sortingModeActive = sorting; }

    public static double getActiveServoSpeed() {
        if (sortingModeActive) return SpindexerDelays.sortingServoSpeed;
        if (transferActive) {
            return Flywheel.getVelocity() >= SpindexerDelays.shootVelocityThreshold
                    ? SpindexerDelays.shootHighVelSpeed
                    : SpindexerDelays.shootLowVelSpeed;
        }
        return SpindexerDelays.intakeServoSpeed;
    }

    public static int getStep() { return currentPort; }
    public static double getCurrentPos() { return currentPos; }
    public static boolean isTransferActive() { return transferActive; }
    public static String[] getBallSlots() { return ports; }
    public static String[] getCurrentPattern() { return currentPattern; }
    public static double getTransferMotorPower() { return transferMotor.getPower(); }
    public static double getTransferTimeLeft() {
        if (!transferActive) return 0;
        return Math.max(0, SpindexerDelays.transferDurationMs - (timer.milliseconds() - transferStartTime));
    }

    public static String formatBallSlots() {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < 3; i++) {
            sb.append(ports[i] == null ? "empty" : "unknown".equals(ports[i]) ? "???" : ports[i]);
            if (i < 2) sb.append(", ");
        }
        return sb.append("]").toString();
    }

    public static String formatShootingOrder() {
        if (getBallCount() == 0) return "No balls";
        StringBuilder sb = new StringBuilder();
        boolean first = true;
        for (int i = 0; i < 3; i++) {
            if (ports[i] != null) {
                if (!first) sb.append(" > ");
                sb.append("unknown".equals(ports[i]) ? "???" : ports[i]);
                first = false;
            }
        }
        return sb.toString();
    }

    public static void stopAll() {
        transferMotor.setPower(0);
        setPosition(SpindexerPositions.initPosition);
    }

    // Legacy compatibility
    public static int getPosition() { return 0; }
    public static int getTargetPosition() { return 0; }
    public static void clearPorts() { clearBalls(); }
    public static String[] getPorts() { return ports; }
    public static void turn(int turns) { if (turns > 0) advanceSpindexer(); else reverseSpindexer(); }
    public static void updatePorts(String color) { ports[0] = color; }
    public static boolean isBusy() { return !isSettled(); }
    public static void enterShootingMode() { rotateForShooting(false); }
    public static void exitShootingMode() { }
    public static void setManualPower(double power) { }
    public static void syncTarget() { }
    public static boolean isManualMode() { return false; }
    public static void update() { updateSlew(); }
    public static void stop() { stopAll(); }
    public static void setStart() { clearBalls(); }
}

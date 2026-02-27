package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tickle {
    private static Servo tickle;
    private static boolean flicked;

    // Servo positions
    private static final double minPosition = 0.18;
    private static final double maxPosition = 1.0; //0.87
    private static final double transferPosition = 0.63;
    // Auto-transfer timing (ms)
    public static final double flickUpWaitMs = 250;
    public static final double flickDownWaitMs = 250;
    public static final double postSpinWaitMs = 175;

    // Servo control
    private static double servoSpeed = 1.0;
    private static double positionTolerance = 0.02;
    private static double currentPosition = minPosition;

    public static void init(HardwareMap map) {
        tickle = map.get(Servo.class, "servo");
        currentPosition = minPosition;
        retract();
    }

    public static void flick() {
        currentPosition = maxPosition;
        tickle.setPosition(maxPosition);
        flicked = true;
    }

    public static void retract() {
        currentPosition = minPosition;
        tickle.setPosition(minPosition);
        flicked = false;
    }

    public static void transfer() {
        currentPosition = transferPosition;
        tickle.setPosition(transferPosition);
        flicked = false;
    }

    // Incremental position control (for trigger-based control)
    public static void incrementPosition(double triggerValue) {
        currentPosition += servoSpeed * triggerValue * 0.02; // Scale for smooth movement
        currentPosition = Math.min(maxPosition, currentPosition);
        tickle.setPosition(currentPosition);
        flicked = currentPosition >= maxPosition - positionTolerance;
    }

    public static void decrementPosition(double triggerValue) {
        currentPosition -= servoSpeed * triggerValue * 0.02; // Scale for smooth movement
        currentPosition = Math.max(minPosition, currentPosition);
        tickle.setPosition(currentPosition);
        flicked = false;
    }

    // Direct position control
    public static void setPosition(double position) {
        currentPosition = Math.max(minPosition, Math.min(maxPosition, position));
        tickle.setPosition(currentPosition);
        flicked = currentPosition >= maxPosition - positionTolerance;
    }

    public static double getPosition() {
        return tickle.getPosition();
    }

    public static double getCurrentPosition() {
        return currentPosition;
    }

    public static boolean isAtPosition(double targetPosition) {
        return Math.abs(tickle.getPosition() - targetPosition) <= positionTolerance;
    }

    public static boolean isAtMax() {
        return isAtPosition(maxPosition);
    }

    public static boolean isAtMin() {
        return isAtPosition(minPosition);
    }

    public static boolean getStatus() {
       return flicked;
    }

    // Getters for position constants
    public static double getMinPosition() {
        return minPosition;
    }

    public static double getMaxPosition() {
        return maxPosition;
    }

    public static double getTransferPosition() {
        return transferPosition;
    }

    public static double getPositionTolerance() {
        return positionTolerance;
    }
}

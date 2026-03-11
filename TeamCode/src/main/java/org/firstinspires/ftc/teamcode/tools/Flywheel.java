package org.firstinspires.ftc.teamcode.tools;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {

    @Configurable
    public static class FlywheelPID {
        public static double kP = 0.02;
        public static double kI = 0.005;
        public static double kD = 0.0;
        public static double kF = 0.00041;
        public static double targetVelocity = 0;
        public static double integralMax = 0.3;
        public static double closeRangeVelocity = 1250;
        public static double longRangeVelocity = 1450;
    }

    @Configurable
    public static class FlywheelEquation {
        public static double tyA = -34.21877;
        public static double tyB = 1178.87689;
        public static double txA = 1.0;
        public static double txB = 2.0;
        public static double txC = 1250.0;
        public static double minVelocity = 1200;
        public static double maxVelocity = 1600;
        public static double roundTo = 5.0;
    }

    private static DcMotorEx motor;
    private static double integralSum = 0;
    private static double lastError = 0;
    private static boolean rumbled = false;

    public static void init(HardwareMap map) {
        motor = map.get(DcMotorEx.class, "shooter");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        integralSum = 0;
        lastError = 0;
        rumbled = false;
    }

    public static void update(double dt) {
        double velocity = motor.getVelocity();
        double error = FlywheelPID.targetVelocity - velocity;
        if (FlywheelPID.targetVelocity == 0) {
            integralSum = 0;
            lastError = 0;
            motor.setPower(0);
            rumbled = false;
            return;
        }
        integralSum = clamp(integralSum + error * dt, -FlywheelPID.integralMax, FlywheelPID.integralMax);
        double output = (FlywheelPID.kP * error)
                + (FlywheelPID.kI * integralSum)
                + (FlywheelPID.kD * (error - lastError) / dt)
                + (FlywheelPID.kF * FlywheelPID.targetVelocity);
        motor.setPower(clamp(output, -1.0, 1.0));
        lastError = error;
    }

    public static double getVelocity() { return motor.getVelocity(); }
    public static double getError() { return FlywheelPID.targetVelocity - motor.getVelocity(); }
    public static double getPower() { return motor.getPower(); }

    public static boolean isAtSpeed(double tolerance) {
        return FlywheelPID.targetVelocity != 0 && Math.abs(getError()) < tolerance;
    }

    public static boolean checkRumble(double tolerance) {
        if (isAtSpeed(tolerance) && !rumbled) {
            rumbled = true;
            return true;
        }
        return false;
    }

    public static void resetRumble() { rumbled = false; }

    public static void stop() {
        FlywheelPID.targetVelocity = 0;
        integralSum = 0;
        lastError = 0;
        motor.setPower(0);
    }

    public static double calculateFromTy(double ty) {
        double raw = FlywheelEquation.tyA * ty + FlywheelEquation.tyB;
        raw = clamp(raw, FlywheelEquation.minVelocity, FlywheelEquation.maxVelocity);
        return Math.round(raw / FlywheelEquation.roundTo) * FlywheelEquation.roundTo;
    }

    public static double calculateFromTx(double tx) {
        double raw = FlywheelEquation.txA * tx * tx + FlywheelEquation.txB * tx + FlywheelEquation.txC;
        raw = clamp(raw, FlywheelEquation.minVelocity, FlywheelEquation.maxVelocity);
        return Math.round(raw / FlywheelEquation.roundTo) * FlywheelEquation.roundTo;
    }

    // Legacy compatibility
    public static double closeRangeVelocity = 1250;
    public static double longRangeVelocity = 1450;
    public static void update() { update(0.02); }
    public static void setTargetVelocity(double v) { FlywheelPID.targetVelocity = v; }
    public static void setTargetVelocity(int v) { FlywheelPID.targetVelocity = v; }
    public static double calculateTargetVelocity(double ty) { return calculateFromTy(ty); }
    public static void resetPID() { integralSum = 0; lastError = 0; }
    public static void setGains(double p, double i, double d, double f) {
        FlywheelPID.kP = p; FlywheelPID.kI = i; FlywheelPID.kD = d; FlywheelPID.kF = f;
    }
    public static void run(double power) { motor.setPower(power); }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
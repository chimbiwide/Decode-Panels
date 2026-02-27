package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {
    private static DcMotorEx wheel;

    // PID gains
    private static double kP = 0.0037;
    private static double kI = 0.0;
    private static double kD = 0.000008;
    private static double kF = 0.00041;  // Feedforward gain

    // PID state
    private static double targetVelocity = 0;  // ticks per second
    private static double integral = 0;
    private static double lastError = 0;
    private static ElapsedTime timer = new ElapsedTime();
    private static double lastTime = 0;

    // Anti-windup limit
    private static double integralMax = 0.3;

    public static void init(HardwareMap map) {
        wheel = map.get(DcMotorEx.class, "shooter");
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        targetVelocity = 0;
        timer.reset();
        lastTime = 0;
        resetPID();
    }

    public static void setGains(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }

    public static void setTargetVelocity(double ticksPerSecond) {
        if (targetVelocity != ticksPerSecond) {
            resetPID();
        }
        targetVelocity = ticksPerSecond;
    }

    public static void update() {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        if (dt <= 0 || dt > 0.5) {
            return;  // Skip invalid time deltas
        }

        double currentVelocity = wheel.getVelocity();  // ticks per second
        double error = targetVelocity - currentVelocity;

        // Proportional
        double pTerm = kP * error;

        // Integral with anti-windup
        integral += error * dt;
        integral = Math.max(-integralMax, Math.min(integralMax, integral));
        double iTerm = kI * integral;

        // Derivative
        double derivative = (error - lastError) / dt;
        double dTerm = kD * derivative;
        lastError = error;

        // Feedforward
        double fTerm = kF * targetVelocity;

        // Calculate output
        double output = pTerm + iTerm + dTerm + fTerm;
        output = Math.max(-1.0, Math.min(1.0, output));

        wheel.setPower(output);
    }

    public static void resetPID() {
        integral = 0;
        lastError = 0;
        lastTime = timer.seconds();
    }

    public static double getVelocity() {
        return wheel.getVelocity();
    }

    public static double getTargetVelocity() {
        return targetVelocity;
    }

    public static double getError() {
        return targetVelocity - wheel.getVelocity();
    }

    public static boolean isAtSpeed(double tolerance) {
        return Math.abs(getError()) < tolerance;
    }

    public static void run(double power) {
        targetVelocity = 0;
        wheel.setPower(power);
    }

    public static void stop() {
        targetVelocity = 0;
        resetPID();
        wheel.setPower(0);
    }

    public static double calculateTargetVelocity(double Ty) {
        //linear equation: -34.21877x + 1178.87689
        return -34.21877 * Ty + 1178.87689;
    }

    // TX-based velocity equation coefficients
    public static double txEqA = 1.0;
    public static double txEqB = 2.0;
    public static double txEqC = 1250.0;
    public static double txEqMinVelocity = 1200;
    public static double txEqMaxVelocity = 1600;
    public static double txEqRoundTo = 25.0;

    // Preset velocities
    public static double closeRangeVelocity = 1250;
    public static double longRangeVelocity = 1450;

    /**
     * Compute flywheel target velocity from Limelight tx using quadratic equation.
     * Result is clamped to [txEqMinVelocity, txEqMaxVelocity] and rounded.
     */
    public static double calculateTargetVelocityFromTx(double tx) {
        double raw = txEqA * tx * tx + txEqB * tx + txEqC;
        raw = Math.max(txEqMinVelocity, Math.min(txEqMaxVelocity, raw));
        return Math.round(raw / txEqRoundTo) * txEqRoundTo;
    }
}

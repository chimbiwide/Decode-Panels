package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {
    private static DcMotor motor;
    //PID
    private static double kP = 0.015; //was 0.005
    private static double kI = 0.0001;
    private static double kD = 0.0008;

    private static int minTicks = -354;
    private static int maxTicks = 188;
    private static double motorPower = 0.6;
    private static int target = 0;

    // PID controller state
    private static double lastTx = 0;
    private static double integralSum = 0;
    private static ElapsedTime timer = new ElapsedTime();
    private static double lastTime = 0;
    // Tracking deadzone (tx values within this range are considered "on target")
    private static double minTx = -0.4;
    private static double maxTx = 0.4;
    private static double integralMax = 0.3;

    /*
    // Wraparound: auto-return when turret exceeds bounds
    public static boolean enableWraparound = true;
    private static boolean wraparoundActive = false;
    private static int wraparoundTarget = 0;
    private static double wraparoundIntegral = 0;
    private static double wraparoundLastError = 0;
    private static double wraparoundStartTime = 0;
    private static double wraparoundCooldownUntil = 0;
    // Wraparound PID tuning
    public static double wraparoundKP = 0.015;
    public static double wraparoundKI = 0.0;
    public static double wraparoundKD = 0.0005;
    public static double wraparoundMaxPower = 0.4;
    public static double wraparoundTolerance = 10.0;
    public static double wraparoundIntegralMax = 0.3;
    public static double wraparoundTimeoutSec = 5.0;
    // Wraparound bounds (turret wraps from one to the other)
    public static int wraparoundMinTicks = -314;
    public static int wraparoundMaxTicks = 1004;

    // Field-centric heading compensation
    // Gear ratio 8:27 (27/8 = 3.375 reduction), motor TPR ~384.5
    // ticksPerRadian = 384.5 * (27.0/8.0) / (2 * Math.PI) ≈ 206.5
    public static double ticksPerRadian = 206.5;
    private static double lastRobotHeading = 0;
    private static boolean fieldCentricEnabled = true;
    private static double headingCompensationPower = 0; // feedforward for tracking mode
    */

    public static void init(HardwareMap map) {
        motor = map.get(DcMotor.class, "turntable");
        timer.reset();
        lastTime = 0;
        lastTx = 0;
        integralSum = 0;
        // Initialize motor in position-hold mode at current position
        target = motor.getCurrentPosition();
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(motorPower);
    }

    /*
    public static void setRobotHeading(double heading) {
        lastRobotHeading = heading;
        headingCompensationPower = 0;
    }

    public static void compensateRotation(double currentHeading) {
        if (!fieldCentricEnabled) return;

        double deltaHeading = MathFunctions.angleWrap(currentHeading - lastRobotHeading);
        lastRobotHeading = currentHeading;

        if (Math.abs(deltaHeading) < 1e-6) {
            headingCompensationPower = 0;
            return;
        }

        int deltaTicks = (int) (-deltaHeading * ticksPerRadian);
        target += deltaTicks;

        // Clamp target to wiring limits
        target = Math.max(maxTicks, Math.min(minTicks, target));

        // When not in tracking mode (RUN_WITHOUT_ENCODER), actively drive to compensated target
        // track() uses RUN_WITHOUT_ENCODER, so this check distinguishes position-hold vs tracking
        if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            motor.setTargetPosition(target);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(motorPower);
        }

        // Store compensation as feedforward power for tracking mode
        headingCompensationPower = -deltaHeading * ticksPerRadian * kP;
        headingCompensationPower = Math.max(-motorPower, Math.min(motorPower, headingCompensationPower));
    }

    public static void setFieldCentricEnabled(boolean enabled) {
        fieldCentricEnabled = enabled;
    }

    public static boolean isFieldCentricEnabled() {
        return fieldCentricEnabled;
    }
    */
    public static void turn(int ticks) {
        target += ticks;
        // Clamp target to wiring limits
        target = Math.max(minTicks, Math.min(maxTicks, target));
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(motorPower);
    }

    public static int getPosition() {
        return motor.getCurrentPosition();
    }

    public static int getTargetPosition() {
        return target;
    }

    public static void stop() {
        motor.setPower(0.0);
    }
    //need AprilTag class
    public static void track(double tx, double ty) {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        dt = Math.max(dt, 1e-3);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Deadzone: stop when tx is within tolerance
        if (tx >= minTx && tx <= maxTx) {
            motor.setPower(0);
            integralSum = 0;
            lastTx = 0;
            return;
        }

        double error = -tx;

        // Reset integral on sign change (prevents overshoot)
        if (lastTx != 0 && Math.signum(error) != Math.signum(lastTx)) {
            integralSum = 0;
        }

        // Integral with anti-windup
        integralSum += error * dt;
        integralSum = Math.max(-integralMax, Math.min(integralMax, integralSum));

        // Derivative
        double derivative = (error - lastTx) / dt;

        // PID output
        double power = (kP * error) + (kI * integralSum) + (kD * derivative);
        power = Math.max(-motorPower, Math.min(motorPower, power));

        // Bounds checking: prevent driving past wiring limits
        // +power → positive ticks (toward maxTicks=256)
        // -power → negative ticks (toward minTicks=-315)
        int turretPos = motor.getCurrentPosition();
        if (turretPos >= maxTicks && power > 0) {

            power = 0;
            integralSum = 0;
        } else if (turretPos <= minTicks && power < 0) {
            power = 0;
            integralSum = 0;
        }

        motor.setPower(power);
        lastTx = error;
    }
    public static boolean autoTrack(double tx, double ty) {
        // Use track() method for consistent PID behavior
        track(tx, ty);
        // Return true if still tracking (not in deadzone)
        return !(tx >= minTx && tx <= maxTx);
    }

    /**
     * Direct power control for manual turret movement (bumpers).
     * Uses RUN_WITHOUT_ENCODER to avoid conflicts with tracking and compensation.
     */
    public static void manualTurn(double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int pos = motor.getCurrentPosition();
        if ((pos >= maxTicks && power > 0) || (pos <= minTicks && power < 0)) {
            motor.setPower(0);
        } else {
            motor.setPower(power);
        }
    }

    /**
     * Call when releasing manual control (or disabling tracking) to sync
     * target to current position and restore position-hold mode.
     */
    public static void syncAfterManual() {
        target = motor.getCurrentPosition();
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(motorPower);
    }

    public static void resetPID() {
        integralSum = 0;
        lastTx = 0;
        lastTime = timer.seconds();
    }

    public static boolean isInDeadzone(double tx) {
        return tx >= minTx && tx <= maxTx;
    }

    /*
    public static boolean updateWraparound(double dt) {
        if (!enableWraparound) return false;

        int currentPos = motor.getCurrentPosition();
        double now = timer.seconds();

        if (!wraparoundActive && now > wraparoundCooldownUntil) {
            if (currentPos > wraparoundMaxTicks) {
                wraparoundActive = true;
                wraparoundTarget = wraparoundMinTicks;
                wraparoundIntegral = 0;
                wraparoundLastError = 0;
                wraparoundStartTime = now;
            } else if (currentPos < wraparoundMinTicks) {
                wraparoundActive = true;
                wraparoundTarget = wraparoundMaxTicks;
                wraparoundIntegral = 0;
                wraparoundLastError = 0;
                wraparoundStartTime = now;
            }
        }

        if (!wraparoundActive) return false;

        double elapsed = now - wraparoundStartTime;

        if (elapsed > wraparoundTimeoutSec) {
            wraparoundActive = false;
            wraparoundIntegral = 0;
            wraparoundLastError = 0;
            motor.setPower(0);
            enableWraparound = false;
            return false;
        }

        double error = wraparoundTarget - currentPos;

        if (Math.abs(error) <= wraparoundTolerance) {
            wraparoundActive = false;
            wraparoundIntegral = 0;
            wraparoundLastError = 0;
            wraparoundCooldownUntil = now + 1.0;
            motor.setPower(0);
            return false;
        }

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wraparoundIntegral += error * dt;
        wraparoundIntegral = Math.max(-wraparoundIntegralMax, Math.min(wraparoundIntegralMax, wraparoundIntegral));

        double derivative = (dt >= 1e-3) ? (error - wraparoundLastError) / dt : 0;

        double output = (wraparoundKP * error) + (wraparoundKI * wraparoundIntegral) + (wraparoundKD * derivative);
        output = Math.max(-wraparoundMaxPower, Math.min(wraparoundMaxPower, output));

        motor.setPower(output);
        wraparoundLastError = error;

        return true;
    }

    public static boolean isWraparoundActive() {
        return wraparoundActive;
    }

    public static void cancelWraparound() {
        wraparoundActive = false;
        wraparoundIntegral = 0;
        wraparoundLastError = 0;
        motor.setPower(0);
    }

    public static void setWraparoundBounds(int min, int max) {
        wraparoundMinTicks = min;
        wraparoundMaxTicks = max;
    }
    */
}

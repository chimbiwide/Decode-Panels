package org.firstinspires.ftc.teamcode.tools;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {

    @Configurable
    public static class TurretTracking {
        public static double kP = 0.02;
        public static double kI = 0.0001;
        public static double kD = 0.0008;
        public static double integralMax = 0.3;
        public static double turretPower = 0.5;
        public static double txDeadzone = 0.5;
        public static boolean enableTracking = false;
    }

    @Configurable
    public static class TurretConfig {
        public static double manualPower = 0.5;
        public static int minTicks = -10000;
        public static int maxTicks = 28200;
        public static boolean enableWraparound = true;
        public static double wraparoundKP = 0.015;
        public static double wraparoundKI = 0.0;
        public static double wraparoundKD = 0.0005;
        public static double wraparoundMaxPower = 0.8;
        public static double wraparoundTolerance = 10.0;
        public static double wraparoundIntegralMax = 0.3;
        public static double wraparoundTimeoutSec = 5.0;
    }

    private static DcMotorEx motor;
    private static final ElapsedTime timer = new ElapsedTime();

    private static double trackingIntegral = 0;
    private static double trackingLastError = 0;

    private static boolean wraparoundActive = false;
    private static int wraparoundTarget = 0;
    private static double wraparoundIntegral = 0;
    private static double wraparoundLastError = 0;
    private static double wraparoundStartTime = 0;

    public static void init(HardwareMap map) {
        motor = map.get(DcMotorEx.class, "turntable");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trackingIntegral = 0;
        trackingLastError = 0;
        wraparoundActive = false;
        timer.reset();
    }

    public static void update(double dt, Double tx, boolean leftBumper, boolean rightBumper) {
        if (TurretConfig.enableWraparound) {
            int pos = motor.getCurrentPosition();
            if (!wraparoundActive) {
                if (pos > TurretConfig.maxTicks) {
                    startWraparound(TurretConfig.minTicks);
                } else if (pos < TurretConfig.minTicks) {
                    startWraparound(TurretConfig.maxTicks);
                }
            }
            if (wraparoundActive) {
                double elapsed = timer.seconds() - wraparoundStartTime;
                boolean cancel = leftBumper && rightBumper;
                if (cancel || elapsed > TurretConfig.wraparoundTimeoutSec) {
                    wraparoundActive = false;
                    wraparoundIntegral = 0;
                    wraparoundLastError = 0;
                    motor.setPower(0);
                    if (elapsed > TurretConfig.wraparoundTimeoutSec) TurretConfig.enableWraparound = false;
                    return;
                }
                double wErr = wraparoundTarget - pos;
                if (Math.abs(wErr) <= TurretConfig.wraparoundTolerance) {
                    wraparoundActive = false;
                    wraparoundIntegral = 0;
                    wraparoundLastError = 0;
                    motor.setPower(0);
                    return;
                }
                wraparoundIntegral = clamp(wraparoundIntegral + wErr * dt,
                        -TurretConfig.wraparoundIntegralMax, TurretConfig.wraparoundIntegralMax);
                double wOut = (TurretConfig.wraparoundKP * wErr)
                        + (TurretConfig.wraparoundKI * wraparoundIntegral)
                        + (TurretConfig.wraparoundKD * (wErr - wraparoundLastError) / dt);
                motor.setPower(clamp(wOut, -TurretConfig.wraparoundMaxPower, TurretConfig.wraparoundMaxPower));
                wraparoundLastError = wErr;
                return;
            }
        }

        if (wraparoundActive) {
            trackingIntegral = 0;
            trackingLastError = 0;
            return;
        }

        if (TurretTracking.enableTracking && tx != null) {
            if (Math.abs(tx) > TurretTracking.txDeadzone) {
                double tErr = tx;
                if (trackingLastError != 0 && Math.signum(tErr) != Math.signum(trackingLastError))
                    trackingIntegral = 0;
                trackingIntegral = clamp(trackingIntegral + tErr * dt,
                        -TurretTracking.integralMax, TurretTracking.integralMax);
                double tOut = (TurretTracking.kP * tErr)
                        + (TurretTracking.kI * trackingIntegral)
                        + (TurretTracking.kD * (tErr - trackingLastError) / dt);
                motor.setPower(-clamp(tOut, -TurretTracking.turretPower, TurretTracking.turretPower));
                trackingLastError = tErr;
            } else {
                motor.setPower(0);
                trackingIntegral = 0;
                trackingLastError = 0;
            }
        } else if (!TurretTracking.enableTracking) {
            if (rightBumper) {
                motor.setPower(TurretConfig.manualPower);
            } else if (leftBumper) {
                motor.setPower(-TurretConfig.manualPower);
            } else {
                motor.setPower(0);
            }
            trackingIntegral = 0;
            trackingLastError = 0;
        } else {
            motor.setPower(0);
            trackingIntegral = 0;
            trackingLastError = 0;
        }
    }

    public static void toggleTracking() {
        TurretTracking.enableTracking = !TurretTracking.enableTracking;
        if (TurretTracking.enableTracking) {
            trackingIntegral = 0;
            trackingLastError = 0;
        } else {
            motor.setPower(0);
        }
    }

    private static void startWraparound(int target) {
        wraparoundActive = true;
        wraparoundTarget = target;
        wraparoundIntegral = 0;
        wraparoundLastError = 0;
        wraparoundStartTime = timer.seconds();
    }

    public static boolean isTrackingEnabled() { return TurretTracking.enableTracking; }
    public static boolean isWraparoundActive() { return wraparoundActive; }
    public static int getWraparoundTarget() { return wraparoundTarget; }
    public static int getPosition() { return motor.getCurrentPosition(); }
    public static double getPower() { return motor.getPower(); }

    public static void stop() { motor.setPower(0); }

    // Legacy compatibility
    public static void track(double tx, double ty) { update(0.02, tx, false, false); }
    public static boolean autoTrack(double tx, double ty) {
        track(tx, ty);
        return Math.abs(tx) > TurretTracking.txDeadzone;
    }
    public static boolean isInDeadzone(double tx) { return Math.abs(tx) <= TurretTracking.txDeadzone; }
    public static void manualTurn(double power) { motor.setPower(power); }
    public static void syncAfterManual() { motor.setPower(0); }
    public static int getTargetPosition() { return motor.getCurrentPosition(); }
    public static void resetPID() { trackingIntegral = 0; trackingLastError = 0; }
    public static void turn(int ticks) { }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
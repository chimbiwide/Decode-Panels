package org.firstinspires.ftc.teamcode.tools;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    @Configurable
    public static class IntakeConfig {
        public static double intakePower = 1.0;
        public static double outtakePower = -1.0;
    }

    private static DcMotorEx motor;
    private static boolean toggled = false;
    private static boolean paused = false;
    private static double pauseUntil = 0;

    public static void init(HardwareMap map) {
        motor = map.get(DcMotorEx.class, "intake");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        toggled = false;
        paused = false;
        pauseUntil = 0;
    }

    public static void toggle() {
        toggled = !toggled;
        if (toggled) Sorter.activateSlot1();
    }

    public static void update(double nowMs, boolean outtakeHeld) {
        if (nowMs > pauseUntil) paused = false;
        if (outtakeHeld) {
            motor.setPower(IntakeConfig.outtakePower);
        } else if (toggled && !paused) {
            motor.setPower(IntakeConfig.intakePower);
        } else {
            motor.setPower(0);
        }
    }

    public static void triggerPause(double nowMs) {
        pauseUntil = nowMs + Sorter.SpindexerDelays.pauseOnDetectMs;
        paused = true;
    }

    public static boolean isToggled() { return toggled; }
    public static void setToggled(boolean on) { toggled = on; }
    public static boolean isPaused() { return paused; }

    public static void stop() {
        motor.setPower(0);
        toggled = false;
    }

    // Legacy compatibility
    public static void intakeBall(double power) { motor.setPower(power); }
    public static void intakeBall(int power) { motor.setPower(power); }
}
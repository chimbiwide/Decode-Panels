package org.firstinspires.ftc.teamcode.tools;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {

    @Configurable
    public static class DriveConfig {
        public static double maxSpeed = 1.0;
        public static double turnSpeed = 1.0;
    }

    private static DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public static void init(HardwareMap map) {
        frontLeft = map.get(DcMotorEx.class, "frontLeft");
        frontRight = map.get(DcMotorEx.class, "frontRight");
        backLeft = map.get(DcMotorEx.class, "backLeft");
        backRight = map.get(DcMotorEx.class, "backRight");
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void drive(double y, double x, double rx) {
        rx *= DriveConfig.turnSpeed;
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / denom * DriveConfig.maxSpeed);
        backLeft.setPower((y - x + rx) / denom * DriveConfig.maxSpeed);
        frontRight.setPower((y - x - rx) / denom * DriveConfig.maxSpeed);
        backRight.setPower((y + x - rx) / denom * DriveConfig.maxSpeed);
    }

    public static void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
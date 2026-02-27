package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.RobotConfigNameable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.control.Odometry;
import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.tools.Color;
import org.firstinspires.ftc.teamcode.tools.Flywheel;
import org.firstinspires.ftc.teamcode.tools.Intake;
import org.firstinspires.ftc.teamcode.tools.Sorter;
import org.firstinspires.ftc.teamcode.tools.Tickle;
import org.firstinspires.ftc.teamcode.tools.Turret;

public class Actuation {
    private static DcMotor[] motors;
    public static Odometry otto;
    private static final double DEADZONE = 0.05;


    public static void setup(HardwareMap map, Pose startPose, Telemetry t) { // Initialize other things as well
         motors = HardwareMapper.getMotors(map);
         otto = new Odometry(new DcMotor[] {motors[1], motors[2], motors[0]});
         otto.resetEncoders();
        RobotMovement.setup(startPose, t);

        Intake.init(map);
        Sorter.init(map);
        Turret.init(map);
        Flywheel.init(map);
        Tickle.init(map);
        Color.init(map);
    }

    //axial = x, vertical
    //lateral = y
    // yaw = heading
    public static void drive(double axial, double lateral, double yaw) {
        // Apply deadzone to prevent drift from joystick noise
        axial = applyDeadzone(axial);
        lateral = applyDeadzone(lateral);
        yaw = applyDeadzone(yaw);

        double max;
        double leftFrontPower  = axial + lateral+yaw;
        double rightFrontPower = axial - lateral-yaw;
        double leftBackPower   = axial - lateral+yaw;
        double rightBackPower  = axial + lateral-yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        motors[3].setPower(leftFrontPower);
        motors[2].setPower(rightFrontPower);
        motors[1].setPower(leftBackPower);
        motors[0].setPower(rightBackPower);
    }

    private static double applyDeadzone(double value) {
        if (Math.abs(value) < DEADZONE) {
            return 0;
        }
        // Scale the remaining range so there's no jump after deadzone
        return Math.signum(value) * (Math.abs(value) - DEADZONE) / (1.0 - DEADZONE);
    }
}

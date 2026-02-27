package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class motor {
    private static DcMotor motor;

    private enum direction {F, R}

    public static void init(DcMotor mt) {
        motor = mt;
    }

    public static void setDir(String input) {
        direction inputEnum = direction.valueOf(input);

        if (inputEnum == direction.F) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (inputEnum == direction.R) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
}

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "LauncherTest")

public class LauncherTest extends LinearOpMode {
    private DcMotor launcher;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        waitForStart();

        while(opModeIsActive()) {
            launcher.setPower(1);
        }
    }

}

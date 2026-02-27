package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.util.Actuation;

@Autonomous(name = "Odometry Test", group = "Test")
public class OdometryTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, new Pose(0, 0, 0), telemetry);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Actuation.otto.updateOdometry();
            Pose pose = Actuation.otto.getPose();

            telemetry.addData("X (in)", "%.2f", pose.getX());
            telemetry.addData("Y (in)", "%.2f", pose.getY());
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.getR()));
            telemetry.addData("Heading (rad)", "%.4f", pose.getR());
            telemetry.update();
        }
    }
}

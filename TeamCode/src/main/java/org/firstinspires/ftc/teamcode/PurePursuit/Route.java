package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement.initPos;
import static org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement.robotPose;
import static org.firstinspires.ftc.teamcode.util.Actuation.otto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.tools.Flywheel;
import org.firstinspires.ftc.teamcode.tools.Sorter;
import org.firstinspires.ftc.teamcode.util.Actuation;
import org.firstinspires.ftc.teamcode.util.MathFunctions;

public class Route {

    private Pose [] pathPoses;

    public Route(Pose [] p) {
        pathPoses = p;
    }

    public void run(double movementSpeed, double turnSpeed) {
        for (int i = 0; i < pathPoses.length; i++) {
            while (MathFunctions.distance(robotPose.getPoint(), pathPoses[i].getPoint()) > 2 || Math.abs(MathFunctions.angleWrap(pathPoses[i].getR()-robotPose.getR())) > Math.toRadians(3)) {
                otto.updateOdometry();
                Flywheel.update();
                Sorter.update();
                robotPose = otto.getPose();
                robotPose = new Pose(robotPose.getX()+initPos.getX(), robotPose.getY()+initPos.getY(), robotPose.getR()+initPos.getR());
                RobotMovement.goToPosition(pathPoses[i], movementSpeed, turnSpeed);
            }
            Actuation.drive(0, 0, 0);
        }
    }

    public void run(double movementSpeed, double turnSpeed, LinearOpMode opMode) {
        for (int i = 0; i < pathPoses.length; i++) {
            while (opMode.opModeIsActive() &&
                   (MathFunctions.distance(robotPose.getPoint(), pathPoses[i].getPoint()) > 2 ||
                    Math.abs(MathFunctions.angleWrap(pathPoses[i].getR()-robotPose.getR())) > Math.toRadians(3))) {
                otto.updateOdometry();
                Flywheel.update();
                Sorter.update();
                robotPose = otto.getPose();
                robotPose = new Pose(robotPose.getX()+initPos.getX(), robotPose.getY()+initPos.getY(), robotPose.getR()+initPos.getR());
                RobotMovement.goToPosition(pathPoses[i], movementSpeed, turnSpeed);
            }
            Actuation.drive(0, 0, 0);
        }
    }

}

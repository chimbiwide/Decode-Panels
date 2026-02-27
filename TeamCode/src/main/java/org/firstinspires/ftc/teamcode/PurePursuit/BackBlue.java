package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.datatypes.Pose;

@Autonomous(name = "SHIFT-Blue", group = "Auto")
public class BackBlue extends AutoBase {

    double angle = Math.toRadians(-90);
    Pose shooting = new Pose(4, 14, 0);
    Pose finalPose = new Pose(15, 14, angle);

    @Override
    public void runOpMode() {
        // Pipeline 8 = blue goals, search left
        initAuto(new Pose(0, 0, 0), 8);
        searchDirection = 1;
        turretOffset = 0; // Compensate for tilted AprilTag

        waitForStart();

        // Strafe right a little bit to get into shooting range
        driveToWithUpdates(shooting, 0.5, 0.5);
        // === Cycle 1: Shoot 3 preloaded balls ===
        telemetry.addData("Phase", "Shooting preloaded balls");
        telemetry.update();
        shootAllThree();

        //driveToWithUpdates(finalPose, 0.5, 0.5);
        /*
        // === Cycle 2: Drive along first ball row while intaking, then shoot ===
        telemetry.addData("Phase", "Driving + intaking row 1");
        telemetry.update();
        driveToWithUpdates(firstRow, 0.5, 0.5);
        driveAndIntake(firstRowBall, 0.5, 0.5, 10.0); // TODO: TUNE end position

        // Drive back to shooting position
        telemetry.addData("Phase", "Returning to shoot position");
        telemetry.update();
        driveToWithUpdates(shooting, 0.5, 0.2); // TODO: TUNE position

        telemetry.addData("Phase", "Shooting cycle 2");
        telemetry.update();
        shootAllThree();

        // === Cycle 3: Drive along second ball row while intaking, then shoot ===
        telemetry.addData("Phase", "Driving + intaking row 2");
        telemetry.update();
        driveToWithUpdates(secondRow, 0.5, 0.5);
        driveAndIntake(secondRowBall, 0.5, 0.5, 10.0); // TODO: TUNE end position

        // Drive back to shooting position
        telemetry.addData("Phase", "Returning to shoot position");
        telemetry.update();
        driveToWithUpdates(shooting, 0.5, 0.5); // TODO: TUNE position

        telemetry.addData("Phase", "Shooting cycle 3");
        telemetry.update();
        shootAllThree();

        telemetry.addData("Phase", "DONE - 9 balls scored");
        telemetry.update();
         */
    }
}

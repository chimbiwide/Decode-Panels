package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.datatypes.Pose;

//@Autonomous(name = "FrontRed", group = "Auto")
public class FrontRed extends AutoBase {

    @Override
    public void runOpMode() {
        // Pipeline 9 = red goals, search right
        initAuto(new Pose(0, 0, 0), 9);
        searchDirection = 1;

        waitForStart();

        // === Cycle 1: Shoot 3 preloaded balls ===
        telemetry.addData("Phase", "Shooting preloaded balls");
        telemetry.update();
        shootAllThree();

        // === Cycle 2: Drive to first ball row, intake 3, shoot 3 ===
        telemetry.addData("Phase", "Driving to ball row 1");
        telemetry.update();
        driveToWithUpdates(new Pose(20, -15, 0), 0.5, 0.2); // TODO: TUNE position (mirrored X)

        telemetry.addData("Phase", "Intaking balls (row 1)");
        telemetry.update();
        intakeThreeBalls();

        // Drive back to shooting position
        telemetry.addData("Phase", "Returning to shoot position");
        telemetry.update();
        driveToWithUpdates(new Pose(0, 0, 0), 0.5, 0.2); // TODO: TUNE position

        telemetry.addData("Phase", "Shooting cycle 2");
        telemetry.update();
        shootAllThree();

        // === Cycle 3: Drive to second ball row, intake 3, shoot 3 ===
        telemetry.addData("Phase", "Driving to ball row 2");
        telemetry.update();
        driveToWithUpdates(new Pose(20, -30, 0), 0.5, 0.2); // TODO: TUNE position (mirrored X)

        telemetry.addData("Phase", "Intaking balls (row 2)");
        telemetry.update();
        intakeThreeBalls();

        // Drive back to shooting position
        telemetry.addData("Phase", "Returning to shoot position");
        telemetry.update();
        driveToWithUpdates(new Pose(0, 0, 0), 0.5, 0.2); // TODO: TUNE position

        telemetry.addData("Phase", "Shooting cycle 3");
        telemetry.update();
        shootAllThree();

        telemetry.addData("Phase", "DONE - 9 balls scored");
        telemetry.update();
    }
}

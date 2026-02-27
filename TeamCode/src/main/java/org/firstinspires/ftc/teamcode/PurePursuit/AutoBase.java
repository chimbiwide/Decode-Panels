package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.tools.Color;
import org.firstinspires.ftc.teamcode.tools.Flywheel;
import org.firstinspires.ftc.teamcode.tools.Intake;
import org.firstinspires.ftc.teamcode.tools.Sorter;
import org.firstinspires.ftc.teamcode.tools.Tickle;
import org.firstinspires.ftc.teamcode.tools.Turret;
import org.firstinspires.ftc.teamcode.util.Actuation;
import org.firstinspires.ftc.teamcode.util.MathFunctions;

public abstract class AutoBase extends LinearOpMode {

    protected Limelight3A limelight;
    protected double Tx = 0;
    protected double Ty = 0;

    // Search step: how many ticks to turn each loop when searching
    private static final int SEARCH_STEP = 5;

    // Search direction: -1 = left, +1 = right. Set by subclass.
    protected int searchDirection = -1;

    // Turret offset in ticks to compensate for tilted AprilTag. Set by subclass.
    protected int turretOffset = 0;

    protected void initAuto(Pose startPose, int pipeline) {
        Actuation.setup(hardwareMap, startPose, telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(pipeline);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * Aim turret at Limelight target until turret is in deadzone.
     * Turns continuously in searchDirection until the Limelight detects a target,
     * then fine-tracks with PID.
     */
    protected void aimTurret() {
        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && timeout.seconds() < 1.5) {
            Flywheel.update();
            Sorter.update();
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Tx = result.getTx();
                Ty = result.getTy();
                if (!Turret.autoTrack(Tx, Ty)) {
                    break; // In deadzone, turret is aimed
                }
            } else {
                // Turn continuously to search for the tag
                Turret.turn(SEARCH_STEP * searchDirection);
            }
        }
    }

    /**
     * Aim turret and shoot one ball.
     * 1. Aim turret with Limelight (search if needed)
     * 2. Spin flywheel to Ty-based target velocity
     * 3. Wait until at speed
     * 4. Flick, wait, retract
     */
    protected void aimAndShoot() {
        aimTurret();

        // Set flywheel velocity based on distance (Ty)
        double targetVelocity = Flywheel.calculateTargetVelocity(Ty);
        Flywheel.setTargetVelocity(1425); //1400

        // Wait for flywheel to reach speed
        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && !Flywheel.isAtSpeed(20) && timeout.seconds() < 4.0) {
            Flywheel.update();
            Sorter.update();
            // Keep tracking while spinning up
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Tx = result.getTx();
                Ty = result.getTy();
                Turret.autoTrack(Tx, Ty);
            }
        }
        // Fire
        Tickle.flick();
        sleep(1000);
        Tickle.retract();
    }

    /**
     * Shoot all 3 balls from the sorter.
     * Enters shooting mode, fires 3 times with sorter advancement, then exits shooting mode.
     */
    protected void shootAllThree() {
        Sorter.enterShootingMode();

        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            aimAndShoot();
            sleep(1000);

            if (i < 2) {
                // Advance sorter to next ball
                Sorter.turn(1);
                // Wait for sorter to reach position
                ElapsedTime sorterTimeout = new ElapsedTime();
                while (opModeIsActive() && Sorter.isBusy() && sorterTimeout.seconds() < 2.0) {
                    Sorter.update();
                    Flywheel.update();
                }
            }
        }

        Sorter.exitShootingMode();
        Flywheel.setTargetVelocity(0);
    }

    /**
     * Intake 3 balls with color detection.
     * Runs intake motor and uses Color sensor to detect rising edges.
     * Advances sorter on each ball detection.
     */
    protected void intakeThreeBalls() {
        int detectedBalls = 0;
        boolean lastDetected = false;
        Intake.intakeBall(0.8);

        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && detectedBalls < 3 && timeout.seconds() < 8.0) {
            Flywheel.update();
            Sorter.update();

            String color = Color.getColor();
            boolean detected = (color != null);

            // Rising edge: no ball last loop, ball this loop
            if (detected && !lastDetected) {
                detectedBalls++;
                Sorter.updatePorts(color);
                if (detectedBalls < 3) {
                    Sorter.turn(1);
                }
                telemetry.addData("Balls Detected", detectedBalls);
                telemetry.update();
            }

            lastDetected = detected;
        }

        Intake.stop();
    }

    /**
     * Drive to a target pose while keeping PID controllers alive.
     * Uses Route with opModeIsActive() safety check.
     */
    protected void driveToWithUpdates(Pose target, double moveSpeed, double turnSpeed) {
        Route route = new Route(new Pose[]{target});
        route.run(moveSpeed, turnSpeed, this);
    }

    /**
     * Drive along a row of balls while intaking.
     * Drives from the current position toward endPose while running the intake
     * and detecting balls with the color sensor. Stops when 3 balls are collected,
     * the robot reaches the end of the row, or the timeout expires.
     *
     * @param endPose     End of the ball row to drive toward
     * @param moveSpeed   Drive speed (0-1)
     * @param turnSpeed   Turn speed (0-1)
     * @param timeoutSec  Max time before giving up
     */
    protected void driveAndIntake(Pose endPose, double moveSpeed, double turnSpeed, double timeoutSec) {

        int detectedBalls = 0;
        boolean lastDetected = false;
        Intake.intakeBall(0.8);

        ElapsedTime timeout = new ElapsedTime();

        while (opModeIsActive() && detectedBalls < 3 && timeout.seconds() < timeoutSec) {
            // Drive toward end of row
            Actuation.otto.updateOdometry();
            Pose robotPose = Actuation.otto.getPose();
            Pose worldPose = new Pose(
                    robotPose.getX() + RobotMovement.initPos.getX(),
                    robotPose.getY() + RobotMovement.initPos.getY(),
                    robotPose.getR() + RobotMovement.initPos.getR());
            RobotMovement.goToPosition(endPose, moveSpeed, turnSpeed);

            // Check if we've reached the end of the row
            boolean atEnd = MathFunctions.distance(worldPose.getPoint(), endPose.getPoint()) < 2
                    && Math.abs(MathFunctions.angleWrap(endPose.getR() - worldPose.getR())) < Math.toRadians(3);

            Flywheel.update();
            Sorter.update();

            // Ball detection
            String color = Color.getColor();
            boolean detected = (color != null);

            if (detected && !lastDetected) {
                detectedBalls++;
                Sorter.updatePorts(color);
                if (detectedBalls < 3) {
                    Sorter.turn(1);
                }
                telemetry.addData("Balls Detected", detectedBalls);
                telemetry.update();
            }
            lastDetected = detected;

            // Stop driving if we reached the end but keep intaking if balls remain
            if (atEnd) {
                Actuation.drive(0, 0, 0);
            }
        }

        Actuation.drive(0, 0, 0);
        Intake.stop();
    }

    /**
     * Wait for human player to feed balls via intake with color detection.
     * Similar to intakeThreeBalls but with a longer timeout for human player.
     */
    protected void waitForHumanFeed(int ballCount) {
        int detectedBalls = 0;
        boolean lastDetected = false;
        Intake.intakeBall(0.8);

        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && detectedBalls < ballCount && timeout.seconds() < 15.0) {
            Flywheel.update();
            Sorter.update();

            String color = Color.getColor();
            boolean detected = (color != null);

            if (detected && !lastDetected) {
                detectedBalls++;
                Sorter.updatePorts(color);
                if (detectedBalls < ballCount) {
                    Sorter.turn(1);
                }
                telemetry.addData("Balls Fed", detectedBalls + "/" + ballCount);
                telemetry.update();
            }

            lastDetected = detected;
        }

        Intake.stop();
    }
}

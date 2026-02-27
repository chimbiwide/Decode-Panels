package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Sorter Bounds Test")
public class SorterTestBounds extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor sorter = hardwareMap.get(DcMotor.class, "spindexer");
        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double power = 0.3;
        boolean spinning = false;
        boolean lastRightBumper = false;
        int startPosition = 0;
        int fullRotationTicks = 0;
        boolean completed = false;

        telemetry.addLine("Right Bumper: spin one full rotation");
        telemetry.addLine("Hold bumper to spin, release when back to start");
        telemetry.addLine("Dpad Up/Down: adjust power");
        telemetry.addLine("Cross: reset encoder");
        telemetry.update();

        waitForStart();

        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastCross = false;

        while (opModeIsActive()) {
            // Adjust power with dpad
            if (gamepad1.dpad_up && !lastDpadUp) power = Math.min(1.0, power + 0.05);
            if (gamepad1.dpad_down && !lastDpadDown) power = Math.max(0.05, power - 0.05);
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // Reset encoder with cross
            if (gamepad1.cross && !lastCross) {
                sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                completed = false;
            }
            lastCross = gamepad1.cross;

            // Hold right bumper to spin, release to stop and measure
            boolean rightBumper = gamepad1.right_bumper;

            if (rightBumper && !lastRightBumper) {
                // Started pressing — record start position
                startPosition = sorter.getCurrentPosition();
                spinning = true;
                completed = false;
            }

            if (spinning && rightBumper) {
                // While held, run motor
                sorter.setPower(power);
            }

            if (!rightBumper && lastRightBumper && spinning) {
                // Released — stop and measure
                sorter.setPower(0);
                fullRotationTicks = sorter.getCurrentPosition() - startPosition;
                spinning = false;
                completed = true;
            }

            lastRightBumper = rightBumper;

            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Current Position", sorter.getCurrentPosition());
            telemetry.addData("Spinning", spinning);

            if (spinning) {
                telemetry.addData("Ticks so far", sorter.getCurrentPosition() - startPosition);
            }

            if (completed) {
                telemetry.addLine("--- LAST FULL ROTATION ---");
                telemetry.addData("Full Rotation Ticks", fullRotationTicks);
                telemetry.addData("Ticks per Port (÷3)", fullRotationTicks / 3.0);
            }

            telemetry.update();
        }

        sorter.setPower(0);
    }
}

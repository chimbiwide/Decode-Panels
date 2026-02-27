package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret Bounds Test")
public class TurretBoundsTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "spindexer");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double power = 0.3;
        int minSeen = 0;
        int maxSeen = 0;

        telemetry.addLine("Turret Bounds Test");
        telemetry.addLine("Right bumper: turn +power");
        telemetry.addLine("Left bumper: turn -power");
        telemetry.addLine("Release: stop");
        telemetry.addLine("Dpad Up/Down: adjust power");
        telemetry.addLine("Cross (X): reset encoder to 0");
        telemetry.update();

        waitForStart();

        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastCross = false;

        while (opModeIsActive()) {
            // Adjust power
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            if (dpadUp && !lastDpadUp) power = Math.min(power + 0.05, 1.0);
            if (dpadDown && !lastDpadDown) power = Math.max(power - 0.05, 0.05);
            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;

            // Reset encoder
            boolean cross = gamepad1.cross;
            if (cross && !lastCross) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                minSeen = 0;
                maxSeen = 0;
            }
            lastCross = cross;

            // Drive turret
            if (gamepad1.right_bumper) {
                turret.setPower(power);
            } else if (gamepad1.left_bumper) {
                turret.setPower(-power);
            } else {
                turret.setPower(0);
            }

            // Track min/max
            int pos = turret.getCurrentPosition();
            if (pos < minSeen) minSeen = pos;
            if (pos > maxSeen) maxSeen = pos;

            // Direction labels
            String direction = "STOPPED";
            if (gamepad1.right_bumper) direction = "+power (" + String.format("%.2f", power) + ")";
            if (gamepad1.left_bumper) direction = "-power (" + String.format("%.2f", -power) + ")";

            telemetry.addData("Position", pos);
            telemetry.addData("Direction", direction);
            telemetry.addData("Power Setting", String.format("%.2f", power));
            telemetry.addLine("---");
            telemetry.addData("Min seen", minSeen);
            telemetry.addData("Max seen", maxSeen);
            telemetry.addLine("---");
            telemetry.addLine("Turn turret to EACH physical limit.");
            telemetry.addLine("Note which bumper and tick value at each end.");
            telemetry.addLine("Then we know: +power goes toward __ ticks");
            telemetry.update();
        }

        turret.setPower(0);
    }
}

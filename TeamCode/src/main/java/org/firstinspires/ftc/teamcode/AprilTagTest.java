package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.tools.Turret;

@Autonomous
public class AprilTagTest extends OpMode {
    private Limelight3A limelight;

    // Pipeline 8 = Blue goals
    private final int pipeline = 8;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);

        Turret.init(hardwareMap);

        telemetry.addLine("Blue goal tracking initialized (pipeline 8)");
    }

    @Override
    public void start() {
        limelight.start();
        Turret.resetPID();
    }

    @Override
    public void loop() {
        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()) {
            double tx = llresult.getTx();
            double ty = llresult.getTy();
            double ta = llresult.getTa();

            boolean tracking = Turret.autoTrack(tx, ty);

            telemetry.addData("Status", tracking ? "TRACKING" : "ON TARGET");
            telemetry.addData("Tx", "%.2f", tx);
            telemetry.addData("Ty", "%.2f", ty);
            telemetry.addData("Area", "%.4f", ta);
            telemetry.addData("Turret Pos", Turret.getPosition());
            telemetry.addData("In Deadzone", Turret.isInDeadzone(tx));
        } else {
            telemetry.addLine("No blue goal detected");
            Turret.stop();
            Turret.resetPID();
        }
    }

    @Override
    public void stop() {
        Turret.stop();
        limelight.stop();
    }
}

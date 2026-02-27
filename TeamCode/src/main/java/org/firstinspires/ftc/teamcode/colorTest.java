package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.tools.Color;

@Autonomous
public class colorTest extends OpMode {
    private ColorSensor color = null;

    @Override
   public void init() {
        color = hardwareMap.get(ColorSensor.class, "color");
    }

    @Override
    public void loop() {
        // RBG value for purple:

    }
}

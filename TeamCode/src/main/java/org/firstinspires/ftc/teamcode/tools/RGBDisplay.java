package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RGBDisplay {

    private static final double WHITE  = 1.0;
    private static final double BLUE   = 0.611;
    private static final double GREEN  = 0.5;
    private static final double VIOLET = 0.722;

    private static Servo led1, led2, led3;

    public static void init(HardwareMap map) {
        led1 = map.get(Servo.class, "rgb1");
        led2 = map.get(Servo.class, "rgb2");
        led3 = map.get(Servo.class, "rgb3");
        led1.setPosition(WHITE);
        led2.setPosition(WHITE);
        led3.setPosition(WHITE);
    }

    public static void update(boolean sortingMode) {
        double pos;
        if (Sorter.getBallCount() == 0) {
            pos = WHITE;
        } else if (!sortingMode) {
            pos = BLUE;
        } else {
            boolean green = Color.isCS1Green() || Color.isCS2Green();
            boolean purple = Color.isCS1Purple() || Color.isCS2Purple();
            pos = green ? GREEN : purple ? VIOLET : WHITE;
        }
        led1.setPosition(pos);
        led2.setPosition(pos);
        led3.setPosition(pos);
    }
}

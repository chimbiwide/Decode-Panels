package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RGBDisplay {

    private static final double OFF    = 0.0;
    private static final double RED    = 0.277;
    private static final double GREEN  = 0.5;
    private static final double BLUE   = 0.611;
    private static final double VIOLET = 0.722;
    private static final double WHITE  = 1.0;
    private static final double BLINK_INTERVAL_MS = 250;

    private static Servo led1, led2, led3;
    private static final ElapsedTime blinkTimer = new ElapsedTime();
    private static boolean blinkOn = true;

    public static void init(HardwareMap map) {
        led1 = map.get(Servo.class, "rgb1");
        led2 = map.get(Servo.class, "rgb2");
        led3 = map.get(Servo.class, "rgb3");
        led1.setPosition(OFF);
        led2.setPosition(OFF);
        led3.setPosition(OFF);
        blinkTimer.reset();
        blinkOn = true;
    }

    public static void update(boolean jammed, boolean blinking) {
        if (blinking) {
            if (blinkTimer.milliseconds() >= BLINK_INTERVAL_MS) {
                blinkOn = !blinkOn;
                blinkTimer.reset();
            }
            if (!blinkOn) {
                led1.setPosition(OFF);
                led2.setPosition(OFF);
                led3.setPosition(OFF);
                return;
            }
        } else {
            blinkOn = true;
        }

        if (jammed) {
            led1.setPosition(RED);
            led2.setPosition(RED);
            led3.setPosition(RED);
            return;
        }
        if (Sorter.isTransferActive()) {
            led1.setPosition(WHITE);
            led2.setPosition(WHITE);
            led3.setPosition(WHITE);
            return;
        }
        String[] slots = Sorter.getBallSlots();
        setLed(led1, slots[0]);
        setLed(led2, slots[1]);
        setLed(led3, slots[2]);
    }

    private static void setLed(Servo led, String ball) {
        if (ball == null) {
            led.setPosition(WHITE);
        } else if ("green".equals(ball)) {
            led.setPosition(GREEN);
        } else if ("purple".equals(ball)) {
            led.setPosition(VIOLET);
        } else {
            led.setPosition(BLUE);
        }
    }
}

package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class Color {
    private static ColorSensor color;
    private static ColorSensor color2;
    private static boolean hasSensor2 = false;
    private static int[] rgb;
    private static int[] rgb2;
    private static String ballColor;

    // Sensor 1: Range-based detection (min AND max bounds)
    //TODO scale all values down by 1/4
    private static final int purpleRedMin = 250;
    private static final int purpleGreenMin = 250;
    private static final int purpleBlueMin = 250;

    private static final int greenRedMin = 250;
    private static final int greenGreenMin = 250;
    private static final int greenBlueMin = 250;

    // Sensor 2: Minimum-threshold only (configurable)
    public static int purple2RedMin = 250;
    public static int purple2GreenMin = 250;
    public static int purple2BlueMin = 250;

    public static int green2RedMin = 250;
    public static int green2GreenMin = 250;
    public static int green2BlueMin = 250;

    public static void init(HardwareMap map) {
        color = map.get(ColorSensor.class, "colorSensor");
        try {
            color2 = map.get(ColorSensor.class, "colorSensor2");
            hasSensor2 = true;
        } catch (Exception e) {
            color2 = null;
            hasSensor2 = false;
        }
    }

    public static void detectColor() {
        rgb = new int[]{color.red(), color.green(), color.blue()};
        if (hasSensor2) {
            rgb2 = new int[]{color2.red(), color2.green(), color2.blue()};
        }
    }

    public static String getColor() {
        detectColor();
        int r = rgb[0], g = rgb[1], b = rgb[2];

        // Sensor 1: range-based detection
        boolean purple1 = r >= purpleRedMin
                && g >= purpleGreenMin
                && b >= purpleBlueMin;
        boolean green1 = r >= greenRedMin
                && g >= greenGreenMin
                && b >= greenBlueMin;

        // Sensor 2: threshold-based detection
        boolean purple2 = false;
        boolean green2 = false;
        if (hasSensor2) {
            int r2 = rgb2[0], g2 = rgb2[1], b2 = rgb2[2];
            purple2 = r2 >= purple2RedMin && g2 >= purple2GreenMin && b2 >= purple2BlueMin && b2 > g2;
            green2 = r2 >= green2RedMin && g2 >= green2GreenMin && b2 >= green2BlueMin && g2 > b2;
        }

        // OR logic: either sensor detecting a ball
        if (purple1 || purple2 || green1 || green2) {
            ballColor = "B";
        } else {
            ballColor = null;
        }
        return getBallColor();
    }

    public static String getBallColor() { return ballColor; }

    public static int[] getRGB() { return rgb; }

    public static int[] getRGB2() { return rgb2; }

    public static boolean hasSensor2() { return hasSensor2; }

    public static boolean checkMotif(String[] motif, String[] received) {
        return Arrays.equals(motif, received);
    }
}

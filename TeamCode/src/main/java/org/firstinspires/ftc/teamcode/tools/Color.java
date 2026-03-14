package org.firstinspires.ftc.teamcode.tools;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class Color {

    @Configurable
    public static class ColorSensor1 {
        public static double gain = 24;
    }

    @Configurable
    public static class ColorSensor2 {
        public static double gain = 24;
    }

    @Configurable
    public static class ColorThresholds {
        public static double greenThreshold = 0.95;
        public static double purpleThreshold = 0.95;
    }

    @Configurable
    public static class ColorConfig {
        public static double colorTimeoutMs = 2000.0;
        public static boolean enableLEDs = true;
        public static boolean enableSensor1 = true;
        public static boolean enableSensor2 = true;
    }

    private static NormalizedColorSensor sensor1;
    private static NormalizedColorSensor sensor2;

    private static double[] lastCS1 = new double[4];
    private static double[] lastCS2 = new double[4];
    private static boolean lastCS1Green, lastCS1Purple;
    private static boolean lastCS2Green, lastCS2Purple;
    private static String lastDetected = null;

    public static void init(HardwareMap map) {
        sensor1 = map.get(NormalizedColorSensor.class, "colorSensor1");
        sensor2 = map.get(NormalizedColorSensor.class, "colorSensor2");
        lastCS1 = new double[4];
        lastCS2 = new double[4];
        lastDetected = null;
        setLEDs(false);
    }

    public static void readSensors() {
        sensor1.setGain((float) ColorSensor1.gain);
        sensor2.setGain((float) ColorSensor2.gain);
        NormalizedRGBA raw1 = sensor1.getNormalizedColors();
        NormalizedRGBA raw2 = sensor2.getNormalizedColors();
        float a1 = Math.max(raw1.alpha, 1e-6f);
        float a2 = Math.max(raw2.alpha, 1e-6f);
        lastCS1 = new double[]{raw1.red / a1, raw1.green / a1, raw1.blue / a1, raw1.alpha};
        lastCS2 = new double[]{raw2.red / a2, raw2.green / a2, raw2.blue / a2, raw2.alpha};
        lastCS1Green = ColorConfig.enableSensor1 && isGreen(lastCS1[0], lastCS1[1], lastCS1[2]);
        lastCS1Purple = ColorConfig.enableSensor1 && isPurple(lastCS1[0], lastCS1[1], lastCS1[2]);
        lastCS2Green = ColorConfig.enableSensor2 && isGreen(lastCS2[0], lastCS2[1], lastCS2[2]);
        lastCS2Purple = ColorConfig.enableSensor2 && isPurple(lastCS2[0], lastCS2[1], lastCS2[2]);
    }

    public static String detectBallColor() {
        readSensors();
        if (lastCS1Green || lastCS2Green) return "green";
        if (lastCS1Purple || lastCS2Purple) return "purple";
        return null;
    }

    public static void updateLastDetected(String color) {
        if (color != null) lastDetected = color;
    }

    public static void setLEDs(boolean on) {
        if (sensor1 instanceof SwitchableLight) ((SwitchableLight) sensor1).enableLight(on);
        if (sensor2 instanceof SwitchableLight) ((SwitchableLight) sensor2).enableLight(on);
    }

    private static boolean isGreen(double r, double g, double b) {
        return g >= ColorThresholds.greenThreshold;
    }

    private static boolean isPurple(double r, double g, double b) {
        return b >= ColorThresholds.purpleThreshold;
    }

    public static double[] getCS1() { return lastCS1; }
    public static double[] getCS2() { return lastCS2; }
    public static boolean isCS1Green() { return lastCS1Green; }
    public static boolean isCS1Purple() { return lastCS1Purple; }
    public static boolean isCS2Green() { return lastCS2Green; }
    public static boolean isCS2Purple() { return lastCS2Purple; }
    public static String getLastDetected() { return lastDetected; }
    public static void clearLastDetected() { lastDetected = null; }

    // Legacy compatibility
    public static String getColor() { return detectBallColor(); }
    public static int[] getRGB() { return new int[]{(int)(lastCS1[0]*255), (int)(lastCS1[1]*255), (int)(lastCS1[2]*255)}; }
    public static int[] getRGB2() { return new int[]{(int)(lastCS2[0]*255), (int)(lastCS2[1]*255), (int)(lastCS2[2]*255)}; }
    public static boolean hasSensor2() { return true; }
    public static String getBallColor() { return lastDetected; }
    public static void detectColor() { readSensors(); }
    public static boolean checkMotif(String[] motif, String[] received) {
        return java.util.Arrays.equals(motif, received);
    }
}
package org.firstinspires.ftc.teamcode.tools;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BallDetector {

    @Configurable
    public static class BallDetection {
        public static boolean useAnalogRanger = true;
        public static double analogThresholdIn = 3.0;
        public static boolean useColorSensorDistance = false;
        public static boolean useColorSensors = true;
        public static double csThresholdMm = 8;
        public static double startupDelayMs = 3000;
        public static double analogSettleDelayMs = 150;
    }

    private static AnalogInput ranger;
    private static DistanceSensor csDistance1;
    private static DistanceSensor csDistance2;

    public static void init(HardwareMap map, NormalizedColorSensor cs1, NormalizedColorSensor cs2) {
        ranger = map.get(AnalogInput.class, "distanceSensor");
        csDistance1 = (cs1 instanceof DistanceSensor) ? (DistanceSensor) cs1 : null;
        csDistance2 = (cs2 instanceof DistanceSensor) ? (DistanceSensor) cs2 : null;
    }

    public static boolean detectBall() {
        boolean triggered = false;
        if (BallDetection.useAnalogRanger) {
            triggered |= getRangerDistanceIn() <= BallDetection.analogThresholdIn;
        }
        if (BallDetection.useColorSensorDistance) {
            if (csDistance1 != null) triggered |= csDistance1.getDistance(DistanceUnit.MM) <= BallDetection.csThresholdMm;
            if (csDistance2 != null) triggered |= csDistance2.getDistance(DistanceUnit.MM) <= BallDetection.csThresholdMm;
        }
        return triggered;
    }

    public static boolean isColorDistanceTriggered() {
        if (!BallDetection.useColorSensorDistance) return false;
        boolean triggered = false;
        if (csDistance1 != null) triggered |= csDistance1.getDistance(DistanceUnit.MM) <= BallDetection.csThresholdMm;
        if (csDistance2 != null) triggered |= csDistance2.getDistance(DistanceUnit.MM) <= BallDetection.csThresholdMm;
        return triggered;
    }

    public static double getRangerDistanceIn() {
        return (ranger.getVoltage() * 48.7) - 4.9;
    }

    public static double getRangerVoltage() {
        return ranger.getVoltage();
    }

    public static double getCS1DistanceMm() {
        return (csDistance1 != null) ? csDistance1.getDistance(DistanceUnit.MM) : -1;
    }

    public static double getCS2DistanceMm() {
        return (csDistance2 != null) ? csDistance2.getDistance(DistanceUnit.MM) : -1;
    }

    public static boolean hasCS1Distance() { return csDistance1 != null; }
    public static boolean hasCS2Distance() { return csDistance2 != null; }
}
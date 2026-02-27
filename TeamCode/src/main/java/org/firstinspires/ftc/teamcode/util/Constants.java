package org.firstinspires.ftc.teamcode.util;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;

public class Constants {
    public static int BACK_RIGHT = 0; // Expansion Hub 2 // right
    public static int BACK_LEFT = 1; // Control Hub 2 // left

    public static int FRONT_RIGHT = 2; // Expansion Hub 3 // back
    public static int FRONT_LEFT = 3; // Control Hub 1
    //public static String[] MOTOR_MAP = {"backRight", "backLeft", "frontRight", "frontLeft"};
    Map<String, Integer> map = new HashMap<String, Integer>()
    {{put("backRight", 0);}};
}
// expansion hub wireing
// encoder port 3 is back odometery
//Front right is port 3
// Back right is port 2
// Spindexer is port 0
// turntable is port 1
//Color sensor I2C port 3
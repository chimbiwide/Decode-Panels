package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.datatypes.Point;

import java.util.List;

public class JohnLimeLight {
    static private Limelight3A limelight;
    private IMU imu;

    private ColorSensor color;

    public enum Alliance {
        BLUE, RED
    }

    public static YawPitchRollAngles getOrientation(IMU imu) {
        return imu.getRobotYawPitchRollAngles();
    }

    public static int getTagID(List<LLResultTypes.FiducialResult> results) {
        int id = 0;
        for (LLResultTypes.FiducialResult fiducial : results) {
            id = fiducial.getFiducialId();
        }
        return id;
    }

    public static double getDistance(double a1, double a2, double h1, double h2) {
        double angToGoalDeg = a1 + a2;
        double angleRadians = angToGoalDeg * (Math.PI / 180.0);
        return (h2 - h1) / Math.tan(angleRadians);
    }

    public static String[] getMotif(int tagId) {
        String[] motif = {};
        if (tagId == 21) {
            motif = new String[] { "G", "P", "P" };
            return motif;
        } else if (tagId == 22) {
            motif = new String[] { "P", "G", "P" };
            return motif;
        } else if (tagId == 23) {
            motif = new String[] { "P", "P", "G" };
            return motif;
        }
        return motif;
    }

    public static Point getPosition(Pose3D botpose) {
        if (botpose != null) {
            double x = botpose.getPosition().x;
            double y = botpose.getPosition().y;
            x *= 39.3701;
            y *= 39.3701;
            return new Point(x, y);
        } else {
            return null;
        }
    }

    public static void trackTag(DcMotor motor, double maxTurnSpeed, double tx) {
        double kP = 0.015;
        double power = -tx * kP;
        power = Math.max(-maxTurnSpeed, Math.min(maxTurnSpeed, power));
        if (Math.abs(tx) < 3.0) {
            motor.setPower(0);
        } else {

            motor.setPower(power);
        }
    }

    public static boolean inZone(Point point) {
        double y = point.getY();
        double x = point.getX();
        if (((y < 72) && (y > Math.abs(x))) || ((y > 72) && (y < -Math.abs(x) - 48))) {
            return true;
        } else {
            return false;
        }
    }

    public static void switchToGoalPipeLine(Limelight3A limelight, Alliance alliance) {
        // pipeline 8 = blue
        // pipeline 9 = red
        limelight.pipelineSwitch(alliance == Alliance.BLUE ? 8 : 9);
    }

    public static void switchToObelisk(Limelight3A limelight) {
        limelight.pipelineSwitch(7);
    }

    public static boolean isAlignedwithTaget(double tx, double tolerance) {
        return Math.abs(tx) <= tolerance;
    }
}
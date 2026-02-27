package org.firstinspires.ftc.teamcode.tools;

import static org.firstinspires.ftc.teamcode.clawtest.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Sorter {
    private static DcMotor sorter;
    static int targetPosition = 0;
    static double stepTicks = 180;
    public static double shootingModeOffset = 90;
    private static boolean inShootingMode = false;
    static int currentPort = 0;
    private static int startPos = 1;
    static String[] ports = new String[3]; // "P", "G", or null

    // PID gains per ball count (index 0 = 0-1 balls, 1 = 2 balls, 2 = 3 balls)
    //0.01
    public static double[] kPs = {0.004, 0.004, 0.004}; //was 0.01
    public static double[] kIs = {0.0001, 0.0001, 0.0001};
    public static double[] kDs = {0.0001, 0.0001, 0.0001};
    private static double integralMax = 0.3;
    private static double maxPower = 1.0;
    private static double positionTolerance = 5.0;

    // PID state
    private static double integralSum = 0;
    private static double lastError = 0;
    private static ElapsedTime timer = new ElapsedTime();
    private static double lastTime = 0;

    public static void init(HardwareMap map) {
       sorter = map.get(DcMotor.class, "spindexer");
       sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // Reset all static state from previous OpMode runs
       targetPosition = 0;
       currentPort = 0;
       inShootingMode = false;
       manualMode = false;
       clearPorts();
       timer.reset();
       lastTime = 0;
       resetPID();
    }

    public static void turn(int turns) {
        targetPosition += (int)(stepTicks * turns);
        // Calculate current port (handle negative modulo correctly)
        currentPort = ((currentPort + turns) % 3 + 3) % 3;
    }

    public static void turnToPort(int port) {
        turn(port - currentPort);
    }

    public static void update() {
        if (manualMode) return; // Skip PID when under direct power control

        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        dt = Math.max(dt, 1e-3);

        int currentPos = sorter.getCurrentPosition();
        double error = targetPosition - currentPos;

        if (Math.abs(error) > positionTolerance) {
            // Select PID gains based on ball count
            int ballCount = getBallCount();
            int pidIndex = Math.max(0, Math.min(2, ballCount - 1)); // 0-1 balls → 0, 2 → 1, 3 → 2
            if (ballCount == 0) pidIndex = 0;
            double kP = kPs[pidIndex];
            double kI = kIs[pidIndex];
            double kD = kDs[pidIndex];

            // Integral with anti-windup
            integralSum += error * dt;
            integralSum = Math.max(-integralMax, Math.min(integralMax, integralSum));

            // Derivative
            double derivative = (error - lastError) / dt;

            // PID output
            double output = (kP * error) + (kI * integralSum) + (kD * derivative);
            output = Math.max(-maxPower, Math.min(maxPower, output));

            sorter.setPower(output);
            lastError = error;
        } else {
            sorter.setPower(0);
            integralSum = 0;
        }
    }

    public static void resetPID() {
        integralSum = 0;
        lastError = 0;
        lastTime = timer.seconds();
    }

    public static void updatePorts(String ballColor) {
        ports[currentPort] = ballColor;
    }

    public static String[] getPorts() {
        return ports;
    }

    public static int getPosition() {
        return sorter.getCurrentPosition();
    }

    public static int getTargetPosition() {
        return targetPosition;
    }

    public static boolean isBusy() {
        return Math.abs(targetPosition - sorter.getCurrentPosition()) > positionTolerance;
    }

    public static int getBallCount() {
        int count = 0;
        for (String item : ports) {
            if (item != null) count++;
        }
        return count;
    }

    public static boolean isFull() {
        for (String item : ports) {
            if (item == null) {
                return false;
            }
        }
        return true;
    }

    public static void enterShootingMode() {
        if (!inShootingMode) {
            targetPosition += (int) shootingModeOffset;
            inShootingMode = true;
        }
    }

    public static void exitShootingMode() {
        if (inShootingMode) {
            targetPosition -= (int) shootingModeOffset;
            inShootingMode = false;
        }
    }

    public static boolean isInShootingMode() {
        return inShootingMode;
    }

    public static void clearPorts() {
        for (int i = 0; i < ports.length; i++) {
            ports[i] = null;
        }
        currentPort = 0;
    }

    // Direct power control bypassing PID. Call syncTarget() when releasing.
    private static boolean manualMode = false;

    public static void setManualPower(double power) {
        manualMode = true;
        sorter.setPower(power);
    }

    public static void syncTarget() {
        // Sync PID target to current position so it doesn't snap back
        targetPosition = sorter.getCurrentPosition();
        manualMode = false;
        resetPID();
    }

    public static boolean isManualMode() {
        return manualMode;
    }

    public static void stop() {
        sorter.setPower(0.0);
        manualMode = false;
        resetPID();
    }

    public static void setStart() {
        // Move to starting position and synchronize internal state
        targetPosition = startPos;
        currentPort = 0;  // Assume start position is port 0
        resetPID();
    }

    public static void organizeFire(String [] ballSequence) {
        int firePort = (currentPort+2)%3;
        targetPosition -= (int)(stepTicks)/2;
        for (int i = 0; i<3;i++) {
            if (ballSequence[i].equals(ports[firePort])) {
                Sorter.update();
                //launch
            }
            else if (ballSequence[i].equals(ports[((firePort-1)%3+3)%3])) {
                turn(1);
                Sorter.update();
                //launch
            }
            else if ((ballSequence[i].equals(ports[(firePort+1)%3]))) {
                turn(-1);
                Sorter.update();
                //launch
            }
            else {
                break;
            }
        }
        targetPosition += (int)(stepTicks)/2;
        currentPort = 0;
        Sorter.update();
    }
    public static void fireAtWill() {
        targetPosition -= (int)(stepTicks)/2;
        turn(1);
        //launch
        // return back to original position and set currentPort to 0
    }
}

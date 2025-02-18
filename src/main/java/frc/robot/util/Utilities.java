package frc.robot.util;

import com.revrobotics.REVLibError;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Utilities {

    /**
     * This method is used to filter driver input.
     * <p>
     * First it applies a deadband to the axis value. Then, it raises the value to a
     * given power, keeping the same sign as the original value.
     * 
     * @param value    The value you want to modify
     * @param power    The exponent you want to raise the value to the power of.
     *                 Can be any positive double, non-integers work too, for
     *                 example 0.5 to take the square root.
     * @param deadband The width of the deadband to apply
     * @return The filtered value
     */
    public static double modifyAxisGeneric(double value, double power, double deadband) {
        // Deadband
        value = MathUtil.applyDeadband(value, deadband);
        // Exponent the input
        value = Math.copySign(Math.pow(Math.abs(value), power), value);
        return value;
    }
    /*
     * I didnt write this and i dont remember what it does
     */
    public static double reboundValue(double value, double anchor){
        double lowerBound = anchor - 180;
        double upperBound = anchor + 180;

        if (value < lowerBound){
            value = lowerBound + ((value-lowerBound)%(upperBound - lowerBound));
        } else if (value > upperBound){
            value = lowerBound + ((value - upperBound)%(upperBound - lowerBound));
        }
        return value;
    }

    public static void verifySparkMaxStatus(REVLibError revResult, int canID, String deviceName, String operation) {
        String alertKey = "SparkMax-" + canID;

        if (revResult != REVLibError.kOk) {
            String text = deviceName + " SparkMax with CAN ID: " + canID + " failed " + operation + "! Result = " + revResult.toString();
            AlertManager.addAlert(alertKey, text, AlertType.kError);
            System.out.println("Error configuring drive motor: " + revResult);
        } else {
            AlertManager.removeAlert(alertKey);
        }
    }
}

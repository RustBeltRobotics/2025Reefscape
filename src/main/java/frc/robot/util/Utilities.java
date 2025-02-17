package frc.robot.util;

import com.revrobotics.REVLibError;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Utilities {
    /**
     * This method is used to filter driver input.
     * <p>
     * First it applies a deadband to the axis value. Then, it squares the value,
     * keeping the same sign as the original value.
     * 
     * @param value The value you want to modify
     * @return The filtered value
     * 
     * FIXME: Matt: I recommend we remove this, as it is arguably obselete by the generic version below.
     */
    public static double modifyAxis(double value) {
        // Deadband
        value = MathUtil.applyDeadband(value, 0.05);
        // Square the axis
        value = Math.copySign(value * value, value);
        return value;
    }

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
     * FIXME: Matt: I recommend we remove this if we don't remember what it does.
     * For what its worth, we're not currently using it.
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
        if (revResult != REVLibError.kOk) {
            //FIXME: Matt - I know nothing about resource leaks, but it sounds bad?
            new Alert(deviceName + " SparkMax with CAN ID: " + canID + " failed " + operation + "! Result = " + revResult.toString(), AlertType.kError).set(true);
            System.out.println("Error configuring drive motor: " + revResult);
        }
    }
}

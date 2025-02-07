package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Rejector extends SubsystemBase {

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public Rejector() {
        motor = new SparkMax(Constants.CanID.REJECTOR_MOTOR, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.inverted(false).idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_ELEVATOR).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_ELEVATOR);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        encoder = motor.getEncoder();
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    public void runAtPercentage(double value) {
        motor.set(value);
    }

    public Command getRejectorOperatorCommand(DoubleSupplier valueSupplier) {
        return this.runEnd(() -> runAtPercentage(valueSupplier.getAsDouble()), () -> stopMotor());
    }
}

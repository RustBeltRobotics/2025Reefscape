package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Rejector extends SubsystemBase {

    private static final double REJECTOR_SPEED = 0.85;
    private static final double REJECTOR_RUN_TIME_SECONDS = 0.75;

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private double desiredSpeed;

    private DoublePublisher speedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Rejector/Speed").publish();

    public Rejector() {
        motor = new SparkMax(Constants.CanID.REJECTOR_MOTOR, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.inverted(false).idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_ELEVATOR).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_ELEVATOR);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        encoder = motor.getEncoder();
    }

    @Override
    public void periodic() {
        speedPublisher.set(desiredSpeed);
    }

    public void stopMotor() {
        motor.stopMotor();
        desiredSpeed = 0.0;
    }

    public void runAtPercentage(double value) {
        desiredSpeed = value;
        motor.set(desiredSpeed);
    }

    public Command getOuttakeCommand() {
        return this.run(() -> runAtPercentage(1.0));
    }

    public Command getIntakeCommand() {
        return this.run(() -> runAtPercentage(-1.0));
    }

    public Command getRejectorOperatorCommand(DoubleSupplier valueSupplier) {
        return this.run(() -> runAtPercentage(valueSupplier.getAsDouble()));
    }

    public Command getStopCommand() {
        return this.runOnce(() -> stopMotor());
    }

    /*
     * Runs the motor at half speed for 0.5 seconds, then stops the motor
     */
    public Command scoreCoralCommand() {
        return this.runEnd(() -> runAtPercentage(REJECTOR_SPEED), () -> stopMotor()).withTimeout(REJECTOR_RUN_TIME_SECONDS);
    }
}

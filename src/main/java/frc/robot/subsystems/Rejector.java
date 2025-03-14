package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Rejector extends SubsystemBase {

    private static final double REJECTOR_SPEED = 0.85;
    private static final double REJECTOR_RUN_TIME_SECONDS = 0.75;

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private boolean leftLaserSensorActive;
    private boolean rightLaserSensorActive;
    private DigitalInput laserSensorLeft;
    private DigitalInput laserSensorRight;

    private double desiredSpeed;

    private DoublePublisher speedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Rejector/Speed").publish();
    private DoublePublisher velocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Rejector/Velocity").publish();
    private DoublePublisher outputCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Rejector/Current").publish();

    private BooleanPublisher leftLaserPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/RBR/Rejector/Laser/Left").publish();
    private BooleanPublisher rightLaserPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/RBR/Rejector/Laser/Right").publish();

    public Rejector() {
        motor = new SparkMax(Constants.CanID.REJECTOR_MOTOR, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.inverted(false).idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_ELEVATOR).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_ELEVATOR);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        encoder = motor.getEncoder();

        laserSensorLeft = new DigitalInput(Constants.DioPort.LASER_SENSOR_LEFT);
        laserSensorRight = new DigitalInput(Constants.DioPort.LASER_SENSOR_RIGHT);
    }

    @Override
    public void periodic() {
        speedPublisher.set(desiredSpeed);
        velocityPublisher.set(encoder.getVelocity());
        leftLaserSensorActive = laserSensorLeft.get();
        rightLaserSensorActive = laserSensorRight.get();
        outputCurrentPublisher.set(motor.getOutputCurrent());
        leftLaserPublisher.set(leftLaserSensorActive);
        rightLaserPublisher.set(rightLaserSensorActive);
    }

    public void stopMotor() {
        motor.stopMotor();
        desiredSpeed = 0.0;
    }

    public void runAtPercentage(double value) {
        desiredSpeed = value;
        motor.set(desiredSpeed);
    }

    public Trigger leftLaserSensorActive() {
        return new Trigger(() -> leftLaserSensorActive);
    }

    public Trigger rightLaserSensorActive() {
        return new Trigger(() -> rightLaserSensorActive);
    }

    public Command getOuttakeCommand() {
        return this.startEnd(() -> runAtPercentage(1.0), () -> stopMotor());
    }

    public Command getOuttakeCommandWithSpeed(double speed) {
        return this.startEnd(() -> runAtPercentage(speed), () -> stopMotor());
    }

    public Command getIntakeCommand() {
        return this.startEnd(() -> runAtPercentage(-1.0), () -> stopMotor());
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

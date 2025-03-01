package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ElevatorTiltPosition;

public class ElevatorTiltMechanism extends SubsystemBase {

    private final SparkMax tiltMotor;  //this is for forward/backward angling motion of the elevator - note: starting position is angled inside robot perimeter
    private final RelativeEncoder tiltEncoder;
    private ElevatorTiltPosition desiredTiltPosition = ElevatorTiltPosition.IN;  //robot always should start with elevator tilted inwards
    private double tiltMotorOutputCurrentAmps;
    private double tiltMotorEncoderPosition;
    private double tiltMotorEncoderVelocity;

    private boolean stallDetected;
    private Debouncer stallDetectionDebouncer = new Debouncer(0.75, DebounceType.kRising);

    private BooleanPublisher tiltMotorStallPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/RBR/Elevator/TiltMotor/Stall").publish();
    private DoublePublisher tiltMotorEncoderPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/TiltMotor/Position").publish();
    private DoublePublisher tiltMotorOutputCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/TiltMotor/Current").publish();
    private DoublePublisher tiltMotorVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/TiltMotor/Velocity").publish();

    public ElevatorTiltMechanism() {
        tiltMotor = new SparkMax(Constants.CanID.ELEVATOR_EXTEND_RETRACT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig tiltConfig = new SparkMaxConfig();
        tiltConfig.inverted(false).idleMode(IdleMode.kBrake);
        tiltConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_ELEVATOR).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_ELEVATOR);
        tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        tiltEncoder = tiltMotor.getEncoder();

        resetTiltEncoder();
    }

    @Override
    public void periodic() { 
        tiltMotorOutputCurrentAmps = tiltMotor.getOutputCurrent();
        tiltMotorEncoderPosition = tiltEncoder.getPosition();
        tiltMotorEncoderVelocity = tiltEncoder.getVelocity();

        tiltMotorOutputCurrentPublisher.set(tiltMotorOutputCurrentAmps);
        tiltMotorEncoderPositionPublisher.set(tiltMotorEncoderPosition);
        tiltMotorVelocityPublisher.set(tiltMotorEncoderVelocity);

        stallDetected = stallDetectionDebouncer.calculate(tiltMotorOutputCurrentAmps > 35);
        tiltMotorStallPublisher.set(stallDetected);
    }

    public Command elevatorTiltXBoxControllerCommand(DoubleSupplier speedSupplier) {
        return this.run(() -> tiltMotor.set(speedSupplier.getAsDouble()));
    }

    public Command toggleElevatorTiltCommand() {
        return new ConditionalCommand(tiltInCommand(), tiltOutCommand(), () -> desiredTiltPosition == ElevatorTiltPosition.OUT);
    }

    public Command tiltOutCommand() {
        //extend the mechanism forwards out beyond the frame perimeter to put elevator in vertical position
        return this.run(() -> {
            desiredTiltPosition = ElevatorTiltPosition.OUT;
            tiltMotor.set(-Constants.Elevator.TILT_MOTOR_OUT_SPEED);
        }).until(
            () -> stallDetected
        ).andThen(
            () -> tiltMotor.set(0.0)
        );
    }

    public Command tiltInCommand() {
        //retract the mechanism back into the frame perimeter 
        return this.run(() -> {
            desiredTiltPosition = ElevatorTiltPosition.IN;
            tiltMotor.set(Constants.Elevator.TILT_MOTOR_IN_SPEED);
        }).until(
            () -> stallDetected
        ).andThen(
            () -> tiltMotor.set(0.0)
        );
    }

    private void resetTiltEncoder() {
        tiltEncoder.setPosition(0.0);
    }
}

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.model.ElevatorPosition;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import java.util.EnumMap;
import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private static ShuffleboardLayout verticalPidValuesEntry = Constants.Shuffleboard.DIAG_TAB
            .getLayout("Elevator vertical PID", BuiltInLayouts.kList)
            .withSize(2, 2);
    private static GenericEntry kPEntry = verticalPidValuesEntry.add("kP", Constants.Elevator.kElevatorKp).getEntry();
    private static GenericEntry kIEntry = verticalPidValuesEntry.add("kI", Constants.Elevator.kElevatorKi).getEntry();
    private static GenericEntry kDEntry = verticalPidValuesEntry.add("kD", Constants.Elevator.kElevatorKd).getEntry();
    private static GenericEntry kSEntry = verticalPidValuesEntry.add("kS", Constants.Elevator.kElevatorKs).getEntry();

    private static Map<ElevatorPosition, Double> elevatorPositionToSetpointMeters;

    //Left/Right are from the perspective of looking forward from the back of the robot towards the front (battery is back side of robot)
    //these are for up/down motion of the elevator
    private final SparkMax leftMotor;
    private final RelativeEncoder leftEncoder;
    private final SparkClosedLoopController leftPidController;
    private final SparkMax rightMotor;
    private final RelativeEncoder rightEncoder;
    private final SparkMax tiltMotor;  //this is for forward/backward angling motion of the elevator
    private final RelativeEncoder tiltEncoder;
    private ElevatorFeedforward verticalFeedForward;
    private double leftMotorOutputCurrentAmps;
    private double leftMotorEncoderPosition;
    private double tiltMotorOutputCurrentAmps;
    private double tiltMotorEncoderPosition;
    private double ksVerticalElevator;
    private double kpVerticalElevator;
    private double kiVerticalElevator;
    private double kdVerticalElevator;

    private DoublePublisher heightPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/Height").publish();
    private DoublePublisher leftMotorOutputCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/LeftMotor/Current").publish();
    private DoublePublisher leftMotorEncoderPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/LeftMotor/Position").publish();
    private DoublePublisher tiltMotorOutputCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/TiltMotor/Current").publish();
    private DoublePublisher tiltMotorEncoderPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/TiltMotor/Position").publish();

    static {
        elevatorPositionToSetpointMeters = new EnumMap<>(ElevatorPosition.class);
       
        elevatorPositionToSetpointMeters.put(ElevatorPosition.BOTTOM, Constants.Elevator.POSITION_BOTTOM);
        elevatorPositionToSetpointMeters.put(ElevatorPosition.L1, Constants.Elevator.POSITION_L1);
        elevatorPositionToSetpointMeters.put(ElevatorPosition.L2, Constants.Elevator.POSITION_L2);
        elevatorPositionToSetpointMeters.put(ElevatorPosition.L3, Constants.Elevator.POSITION_L3);
        elevatorPositionToSetpointMeters.put(ElevatorPosition.L4, Constants.Elevator.POSITION_L4);
    }

    //Note: because the elevator has a cascade design - it will move twice as far as the motor encoder output
    public Elevator() {
        leftMotor = new SparkMax(Constants.CanID.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
        //TODO: confirm if we should be inverting here or not
        SparkMaxConfig leftMotorConfig = getVerticalMotorLeaderConfig(false, Constants.Elevator.kElevatorKp, Constants.Elevator.kElevatorKi, Constants.Elevator.kElevatorKd);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        leftEncoder = leftMotor.getEncoder();
        leftPidController = leftMotor.getClosedLoopController();

        rightMotor = new SparkMax(Constants.CanID.ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig followerConfig = getVerticalMotorFollowerConfig();
        followerConfig.follow(leftMotor, true);
        rightMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightEncoder = rightMotor.getEncoder();

        tiltMotor = new SparkMax(Constants.CanID.ELEVATOR_EXTEND_RETRACT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig tiltConfig = new SparkMaxConfig();
        tiltConfig.inverted(false).idleMode(IdleMode.kBrake);
        tiltConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_ELEVATOR).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_ELEVATOR);
        tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        tiltEncoder = tiltMotor.getEncoder();

        verticalFeedForward = new ElevatorFeedforward(Constants.Elevator.kElevatorKs,  Constants.Elevator.kElevatorKg, Constants.Elevator.kElevatorKv, 
            Constants.Elevator.kElevatorKa);
    
        ksVerticalElevator = Constants.Elevator.kElevatorKs;
        //this assumes the elevator is at the bottom and tilted inwards (inside robot frame) when the robot is powered on
        resetElevatorEncoders();
    }

    public void avoidTipping() {
        //TODO: if elevator is tilted out, snap it back in to vertical position
        //TODO: if elevator is raised high, lower it quickly to bring momentum/CG down
    }

    public Command elevatorTestVerticalSetpointCommand() {
        // return this.run(() -> leftMotor.stopMotor());
        return getSetVerticalGoalCommand(ElevatorPosition.L1);
    }

    public Command elevatorVerticalStopCommand() {
        // return this.run(() -> leftMotor.stopMotor());
        return getSetVerticalGoalCommand(ElevatorPosition.BOTTOM);
    }
/*
    public Command elevatorVerticalXBoxControllerCommand(DoubleSupplier speedSupplier) {
        return this.run(() -> leftMotor.set(speedSupplier.getAsDouble()));
    }

    public Command returnToBottomPosition() {
        //TODO: verify applying negative voltage will move the elevator down
        return this.run(() -> leftMotor.setVoltage(-1.0))
            .until(() -> leftMotorOutputCurrentAmps > 40) //current will jump up when the elevator hits the bottom and cannot move further down
            .finallyDo(() -> resetElevatorEncoders());
    }
*/

    public Command getTestInitialKsCommand() {
        return run(() -> applyKsVoltage());
    }

    @Override
    public void periodic() {
        //update state
        leftMotorOutputCurrentAmps = leftMotor.getOutputCurrent();
        leftMotorEncoderPosition = leftEncoder.getPosition();
        tiltMotorOutputCurrentAmps = tiltMotor.getOutputCurrent();
        tiltMotorEncoderPosition = tiltEncoder.getPosition();

        //publish state to NT
        leftMotorOutputCurrentPublisher.set(leftMotorOutputCurrentAmps);
        leftMotorEncoderPositionPublisher.set(leftMotorEncoderPosition);
        tiltMotorOutputCurrentPublisher.set(tiltMotorOutputCurrentAmps);
        tiltMotorEncoderPositionPublisher.set(tiltMotorEncoderPosition);

        heightPublisher.set(getHeight());

        //this is for initial calculation of kS for ElevatorFeedForward
        double smartDashboardKs = kSEntry.getDouble(Constants.Elevator.kElevatorKs);
        if (ksVerticalElevator != smartDashboardKs) {
            ksVerticalElevator = smartDashboardKs;
            new Alert("Elevator kS updated to: " + ksVerticalElevator, AlertType.kInfo).set(true);
            DataLogManager.log("RBR: Elevator kS updated to: " + ksVerticalElevator);
            verticalFeedForward = new ElevatorFeedforward(ksVerticalElevator, Constants.Elevator.kElevatorKg, Constants.Elevator.kElevatorKv,
                Constants.Elevator.kElevatorKa);
        }

        double smartDashboardKp = kPEntry.getDouble(Constants.Elevator.kElevatorKp);
        double smartDashboardKi = kIEntry.getDouble(Constants.Elevator.kElevatorKi);
        double smartDashboardKd = kDEntry.getDouble(Constants.Elevator.kElevatorKd);
        boolean pidValuesChanged = false;

        if (kpVerticalElevator != smartDashboardKp) {
            kpVerticalElevator = smartDashboardKp;
            new Alert("Elevator kP updated to: " + kpVerticalElevator, AlertType.kInfo).set(true);
            DataLogManager.log("RBR: Elevator kP updated to: " + kpVerticalElevator);
            pidValuesChanged = true;
        }
        if (kiVerticalElevator != smartDashboardKi) {
            kiVerticalElevator = smartDashboardKi;
            new Alert("Elevator kI updated to: " + kiVerticalElevator, AlertType.kInfo).set(true);
            DataLogManager.log("RBR: Elevator kI updated to: " + kiVerticalElevator);
            pidValuesChanged = true;
        }
        if (kdVerticalElevator != smartDashboardKd) {
            kdVerticalElevator = smartDashboardKd;
            new Alert("Elevator kD updated to: " + kdVerticalElevator, AlertType.kInfo).set(true);
            DataLogManager.log("RBR: Elevator kD updated to: " + kdVerticalElevator);
            pidValuesChanged = true;
        }

        if (pidValuesChanged) {
            SparkMaxConfig leaderMotorConfig = getVerticalMotorLeaderConfig(false, kpVerticalElevator, kiVerticalElevator, kdVerticalElevator);
            leftMotor.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    private Command getSetVerticalGoalCommand(ElevatorPosition goalPosition) {
        double goalInMeters = elevatorPositionToSetpointMeters.get(goalPosition);

        return run(() -> reachVerticalGoal(goalInMeters));
    }

    private SparkMaxConfig getVerticalMotorLeaderConfig(boolean invert, double kP, double kI, double kD) {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.inverted(invert).idleMode(IdleMode.kBrake);
        sparkMaxConfig.closedLoopRampRate(0.25);
        sparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(convertDistanceToEncoderRotations(Meters.of(1)).per(Second).in(RPM))
            .maxAcceleration(convertDistanceToEncoderRotations(Meters.of(2)).per(Second).per(Second)
                                    .in(RPM.per(Second)));
        sparkMaxConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_ELEVATOR).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_ELEVATOR);

        return sparkMaxConfig;
    }

    private SparkMaxConfig getVerticalMotorFollowerConfig() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_ELEVATOR).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_ELEVATOR);

        return sparkMaxConfig;
    }

    private void applyKsVoltage() {
        leftMotor.setVoltage(ksVerticalElevator);
    }

    /**
     * Get the height in meters.
     *
     * @return Height in meters
     */
    public double getHeight() {
        return convertEncoderRotationsToDistance(Rotations.of(leftEncoder.getPosition())).in(Meters);
    }

    public void stopVerticalMotors() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public void tiltOut() {
        //TODO: implement - extend the mechanism forwards out beyond the frame perimeter 
    }

    public void tiltIn() {
        //TODO: implement - retract the mechanism back into the frame perimeter 
    }

    /**
     * Run control loop to reach and maintain vertical goal.
     *
     * @param goal the position to maintain (in meters)
     */
    private void reachVerticalGoal(double goalInMeters) {
        leftPidController.setReference(convertDistanceToEncoderRotations(Meters.of(goalInMeters)).in(Rotations),
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0,
            verticalFeedForward.calculate(convertEncoderRotationsToDistance(Rotations.of(leftEncoder.getVelocity())).per(Minute).in(MetersPerSecond))
        );
    }

    private void resetElevatorEncoders() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
        tiltEncoder.setPosition(0.0);
    }

    public static Distance convertEncoderRotationsToDistance(Angle rotations) {
        return Meters.of((rotations.in(Rotations) / Constants.Elevator.kElevatorGearing) * (Constants.Elevator.kElevatorDrumRadius * 2 * Math.PI));
    }

    public static Angle convertDistanceToEncoderRotations(Distance distance) {
      return Rotations.of(distance.in(Meters) / (Constants.Elevator.kElevatorDrumRadius * 2 * Math.PI) * Constants.Elevator.kElevatorGearing);
    }
}

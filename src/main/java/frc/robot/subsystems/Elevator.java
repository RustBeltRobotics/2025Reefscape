package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.model.ElevatorPosition;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements AutoCloseable {

    // private static ShuffleboardLayout verticalPidValuesEntry = Constants.Shuffleboard.DIAG_TAB
    //         .getLayout("Elevator vertical PID", BuiltInLayouts.kList)
    //         .withSize(2, 2);
            
    private static Map<ElevatorPosition, Double> elevatorPositionToSetpointMeters;

    private final Alert debugMsgAlert = new Alert("Elevator debug: ", AlertType.kInfo);

    //Left/Right are from the perspective of looking forward from the back of the robot towards the front (battery is back side of robot)
    //these are for up/down motion of the elevator
    private final SparkMax leftMotor;
    private final RelativeEncoder leftEncoder;
    private final SparkClosedLoopController leftPidController;
    private final SparkMax rightMotor;
    private final RelativeEncoder rightEncoder;
    private final SparkMax tiltMotor;  //this is for forward/backward angling motion of the elevator - note: starting position is angled inside robot perimeter
    private final RelativeEncoder tiltEncoder;
    private ElevatorFeedforward verticalFeedForward;
    private double leftMotorOutputCurrentAmps;
    private double leftMotorEncoderPosition;
    private double leftMotorEncoderVelocity;
    private double tiltMotorOutputCurrentAmps;
    private double tiltMotorEncoderPosition;
    private double tiltMotorEncoderVelocity;
    private double ksVerticalElevator = Constants.Elevator.kElevatorKs;
    private double kpVerticalElevator = Constants.Elevator.kElevatorKp;
    private double kiVerticalElevator = Constants.Elevator.kElevatorKi;
    private double kdVerticalElevator = Constants.Elevator.kElevatorKd;

    private DoublePublisher heightPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/Height").publish();
    
    private DoublePublisher leftMotorEncoderPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/LeftMotor/Position").publish();
    private DoublePublisher leftMotorOutputCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/LeftMotor/Current").publish();
    private DoublePublisher leftMotorVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/LeftMotor/Velocity").publish();
    private DoublePublisher tiltMotorEncoderPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/TiltMotor/Position").publish();
    private DoublePublisher tiltMotorOutputCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/TiltMotor/Current").publish();
    private DoublePublisher tiltMotorVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Elevator/TiltMotor/Velocity").publish();

    // This gearbox represents a gearbox containing 2 NEO motors.
    private DCMotor verticalElevatorSimGearbox = DCMotor.getNEO(2);
    private SparkMaxSim verticalMotorsSim;
    private SparkRelativeEncoderSim verticalEncoderSim;
    private ElevatorSim verticalElevatorSim;

    private final ProfiledPIDController verticalProfiledPidController = new ProfiledPIDController(
        Constants.Elevator.kElevatorKp,
        Constants.Elevator.kElevatorKi,
        Constants.Elevator.kElevatorKd,
        new TrapezoidProfile.Constraints(2.45, 2.45));

    private Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private MechanismLigament2d m_elevatorMech2d;

    static {
        elevatorPositionToSetpointMeters = new EnumMap<>(ElevatorPosition.class);
       
        elevatorPositionToSetpointMeters.put(ElevatorPosition.BOTTOM, Constants.Elevator.POSITION_BOTTOM);
        elevatorPositionToSetpointMeters.put(ElevatorPosition.L1, Constants.Elevator.POSITION_L1);
        elevatorPositionToSetpointMeters.put(ElevatorPosition.L2, Constants.Elevator.POSITION_L2);
        elevatorPositionToSetpointMeters.put(ElevatorPosition.L3, Constants.Elevator.POSITION_L3);
        elevatorPositionToSetpointMeters.put(ElevatorPosition.L4, Constants.Elevator.POSITION_L4);

        SmartDashboard.putNumber("elevatorKp", Constants.Elevator.kElevatorKp);
        SmartDashboard.putNumber("elevatorKi", Constants.Elevator.kElevatorKi);
        SmartDashboard.putNumber("elevatorKd", Constants.Elevator.kElevatorKd);
        SmartDashboard.putNumber("elevatorKs", Constants.Elevator.kElevatorKs);
    }

    public Elevator() {
        leftMotor = new SparkMax(Constants.CanID.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
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

        //this assumes the elevator is at the bottom and tilted inwards (inside robot frame) when the robot is powered on
        resetAllEncoders();

        if (Robot.isSimulation()) {
            verticalElevatorSimGearbox = DCMotor.getNEO(2);
            verticalMotorsSim = new SparkMaxSim(leftMotor, verticalElevatorSimGearbox);
            verticalEncoderSim = verticalMotorsSim.getRelativeEncoderSim();
            verticalElevatorSim = new ElevatorSim(verticalElevatorSimGearbox, Constants.Elevator.kElevatorGearing, Constants.Elevator.kCarriageMass,
                Constants.Elevator.kElevatorDrumRadius, Constants.Elevator.kMinElevatorHeightMeters, Constants.Elevator.kMaxElevatorHeightMeters,
                true, 0, 0.01, 0.0);
            m_elevatorMech2d = m_mech2dRoot.append(new MechanismLigament2d("Elevator", verticalElevatorSim.getPositionMeters(), 90));

            SmartDashboard.putData("Elevator Sim", m_mech2d);
        }

        verticalFeedForward = new ElevatorFeedforward(Constants.Elevator.kElevatorKs,  Constants.Elevator.kElevatorKg, Constants.Elevator.kElevatorKv, 
            Constants.Elevator.kElevatorKa);
    
        ksVerticalElevator = Constants.Elevator.kElevatorKs;
    }

    /** Advance the simulation. */
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        verticalElevatorSim.setInput(verticalMotorsSim.getAppliedOutput() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        verticalElevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        verticalEncoderSim.setPosition(verticalElevatorSim.getPositionMeters());
        verticalEncoderSim.setVelocity(verticalElevatorSim.getVelocityMetersPerSecond());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(verticalElevatorSim.getCurrentDrawAmps()));

        // Update elevator visualization with position
        m_elevatorMech2d.setLength(verticalEncoderSim.getPosition());
    }

    public void avoidTipping() {
        //TODO: if elevator is vertical, snap it back in angled position
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

    public Command elevatorTiltXBoxControllerCommand(DoubleSupplier speedSupplier) {
        return this.run(() -> tiltMotor.set(speedSupplier.getAsDouble()));
    }
/*
    public Command elevatorVerticalXBoxControllerCommand(DoubleSupplier speedSupplier) {
        return this.run(() -> leftMotor.set(speedSupplier.getAsDouble()));
    }

    public Command returnToBottomPosition() {
        //TODO: verify applying negative voltage will move the elevator down
        return this.run(() -> leftMotor.setVoltage(-1.0))
            .until(() -> leftMotorOutputCurrentAmps > Constants.CurrentLimit.SparkMax.SMART_ELEVATOR) //current will jump up when the elevator hits the bottom and cannot move further down
            .finallyDo(() -> resetVerticalElevatorEncoders());
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
        leftMotorEncoderVelocity = leftEncoder.getVelocity();
        tiltMotorOutputCurrentAmps = tiltMotor.getOutputCurrent();
        tiltMotorEncoderPosition = tiltEncoder.getPosition();
        tiltMotorEncoderVelocity = tiltEncoder.getVelocity();

        //publish state to NT
        leftMotorOutputCurrentPublisher.set(leftMotorOutputCurrentAmps);
        leftMotorEncoderPositionPublisher.set(leftMotorEncoderPosition);
        leftMotorVelocityPublisher.set(leftMotorEncoderVelocity);
        tiltMotorOutputCurrentPublisher.set(tiltMotorOutputCurrentAmps);
        tiltMotorEncoderPositionPublisher.set(tiltMotorEncoderPosition);
        tiltMotorVelocityPublisher.set(tiltMotorEncoderVelocity);

        heightPublisher.set(getHeight());

        //this is for initial calculation of kS for ElevatorFeedForward
        double smartDashboardKs = SmartDashboard.getNumber("elevatorKs", Constants.Elevator.kElevatorKs);
        if (ksVerticalElevator != smartDashboardKs) {
            ksVerticalElevator = smartDashboardKs;
            new Alert("Elevator kS updated to: " + ksVerticalElevator, AlertType.kInfo).set(true);
            DataLogManager.log("RBR: Elevator kS updated to: " + ksVerticalElevator);
            verticalFeedForward = new ElevatorFeedforward(ksVerticalElevator, Constants.Elevator.kElevatorKg, Constants.Elevator.kElevatorKv,
                Constants.Elevator.kElevatorKa);
        }

        double smartDashboardKp = SmartDashboard.getNumber("elevatorKp", Constants.Elevator.kElevatorKp);
        double smartDashboardKi = SmartDashboard.getNumber("elevatorKi", Constants.Elevator.kElevatorKi);
        double smartDashboardKd = SmartDashboard.getNumber("elevatorKd", Constants.Elevator.kElevatorKd);
        boolean pidValuesChanged = false;

        if (kpVerticalElevator != smartDashboardKp) {
            kpVerticalElevator = smartDashboardKp;
            verticalProfiledPidController.setP(kpVerticalElevator);
            new Alert("Elevator kP updated to: " + kpVerticalElevator, AlertType.kInfo).set(true);
            DataLogManager.log("RBR: Elevator kP updated to: " + kpVerticalElevator);
            pidValuesChanged = true;
        }
        if (kiVerticalElevator != smartDashboardKi) {
            kiVerticalElevator = smartDashboardKi;
            verticalProfiledPidController.setI(kiVerticalElevator);
            new Alert("Elevator kI updated to: " + kiVerticalElevator, AlertType.kInfo).set(true);
            DataLogManager.log("RBR: Elevator kI updated to: " + kiVerticalElevator);
            pidValuesChanged = true;
        }
        if (kdVerticalElevator != smartDashboardKd) {
            kdVerticalElevator = smartDashboardKd;
            verticalProfiledPidController.setD(kdVerticalElevator);
            new Alert("Elevator kD updated to: " + kdVerticalElevator, AlertType.kInfo).set(true);
            DataLogManager.log("RBR: Elevator kD updated to: " + kdVerticalElevator);
            pidValuesChanged = true;
        }

        // if (pidValuesChanged) {
        //     SparkMaxConfig leaderMotorConfig = getVerticalMotorLeaderConfig(false, kpVerticalElevator, kiVerticalElevator, kdVerticalElevator);
        //     leftMotor.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // }
    }

    private Command getSetVerticalGoalCommand(ElevatorPosition goalPosition) {
        double goalInMeters = elevatorPositionToSetpointMeters.get(goalPosition);

        return run(() -> reachVerticalGoal(goalInMeters));
    }

    private SparkMaxConfig getVerticalMotorLeaderConfig(boolean invert, double kP, double kI, double kD) {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.inverted(invert).idleMode(IdleMode.kBrake);
        sparkMaxConfig.encoder.positionConversionFactor(Constants.Elevator.POSITION_CONVERSION).velocityConversionFactor(Constants.Elevator.VELOCITY_CONVERSION);
        // sparkMaxConfig.closedLoopRampRate(0.25);
        // sparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     .pid(kP, kI, kD)
        //     .outputRange(-1, 1)
        //     .maxMotion
        //     .maxVelocity(convertDistanceToEncoderRotations(Meters.of(1)).per(Second).in(RPM))
        //     .maxAcceleration(convertDistanceToEncoderRotations(Meters.of(2)).per(Second).per(Second).in(RPM.per(Second)));
        sparkMaxConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_ELEVATOR).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_ELEVATOR);

        return sparkMaxConfig;
    }

    private SparkMaxConfig getVerticalMotorFollowerConfig() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.encoder.positionConversionFactor(Constants.Elevator.POSITION_CONVERSION).velocityConversionFactor(Constants.Elevator.VELOCITY_CONVERSION);
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
        return leftEncoder.getPosition();
    }

    public void stopVerticalMotors() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public Command tiltOutCommand() {
        //extend the mechanism forwards out beyond the frame perimeter to put elevator in vertical position
        return this.run(() -> tiltMotor.set(-0.4))
        .until(() -> tiltMotorOutputCurrentAmps > Constants.CurrentLimit.SparkMax.SMART_ELEVATOR && Math.abs(tiltMotorEncoderVelocity) < 0.05);
    }

    public Command tiltInCommand() {
        //retract the mechanism back into the frame perimeter 
        return this.run(() -> tiltMotor.set(0.4))
        .until(() -> tiltMotorOutputCurrentAmps > Constants.CurrentLimit.SparkMax.SMART_ELEVATOR && tiltMotorEncoderVelocity < 0.05);
    }

    /**
     * Run control loop to reach and maintain vertical goal.
     *
     * @param goal the position to maintain (in meters)
     */
    private void reachVerticalGoal(double goalInMeters) {
        verticalProfiledPidController.setGoal(goalInMeters);
        double pidOutput = verticalProfiledPidController.calculate(leftEncoder.getPosition());
        double feedforwardOutput = verticalFeedForward.calculate(verticalProfiledPidController.getSetpoint().velocity);
        leftMotor.setVoltage(pidOutput + feedforwardOutput);

        // leftPidController.setReference(convertDistanceToEncoderRotations(Meters.of(goalInMeters)).in(Rotations),
        //     ControlType.kMAXMotionPositionControl,
        //     ClosedLoopSlot.kSlot0,
        //     verticalFeedForward.calculate(convertEncoderRotationsToDistance(Rotations.of(leftEncoder.getVelocity())).per(Minute).in(MetersPerSecond))
        // );
    }

    private void resetAllEncoders() {
        resetTiltEncoder();
        resetVerticalElevatorEncoders();
    }

    private void resetTiltEncoder() {
        tiltEncoder.setPosition(0.0);
    }

    private void resetVerticalElevatorEncoders() {
        verticalProfiledPidController.setGoal(0.0);
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }
/* 
    public static double getDistanceInMetersForEncoderRotations(double rotations) {
        return rotations * Constants.Elevator.POSITION_CONVERSION * 2;
    }

    public static double getEncoderRotationsForDistanceInMeters(double distance) {
        return distance / Constants.Elevator.POSITION_CONVERSION / 2;
    }

    public static Distance convertEncoderRotationsToDistance(Angle rotations) {
        //Note: the times(2) is because the elevator has a cascade design - it will move twice as far as the motor encoder output
        return Meters.of((rotations.in(Rotations) / Constants.Elevator.kElevatorGearing) * (Constants.Elevator.kElevatorDrumRadius * 2 * Math.PI)).times(2.0);
    }

    public static Angle convertDistanceToEncoderRotations(Distance distance) {
      return Rotations.of(distance.in(Meters) / (Constants.Elevator.kElevatorDrumRadius * 2 * Math.PI) * Constants.Elevator.kElevatorGearing).div(2.0);
    }
*/
    @Override
    public void close() throws Exception {
        m_mech2d.close();
    }
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.model.ElevatorVerticalPosition;
import frc.robot.model.MotorRunMode;
import frc.robot.util.ThresholdValueTimeTracker;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase implements AutoCloseable {

    private static final Map<ElevatorVerticalPosition, Double> elevatorPositionToSetpointMeters;

    // Left/Right are from the perspective of looking forward from the back of the
    // robot towards the front (battery is back side of robot)
    // these are for up/down motion of the elevator
    private final SparkMax leftMotor;
    private final RelativeEncoder leftEncoder;
    private final SparkMax rightMotor;
    private final RelativeEncoder rightEncoder;

    private ElevatorFeedforward verticalFeedForward;
    private double leftMotorOutputCurrentAmps;
    private double leftMotorEncoderPosition;
    private double leftMotorEncoderVelocity;
    private double ksVerticalElevator = Constants.Elevator.kElevatorKs;
    private double kpVerticalElevator = Constants.Elevator.kElevatorKp;
    private double kiVerticalElevator = Constants.Elevator.kElevatorKi;
    private double kdVerticalElevator = Constants.Elevator.kElevatorKd;
    private double currentHeight; // current height as read from encoder (in meters)
    private double goalHeight; // height of the current goal
    private double goalHeightUpperBound = goalHeight + Constants.Elevator.GOAL_DISTANCE_TOLERANCE;
    private double goalHeightLowerBound = goalHeight - Constants.Elevator.GOAL_DISTANCE_TOLERANCE;
    private boolean atGoal; // whether we have reached the desired height yet or not
    private boolean stallDetected;
    private Debouncer stallDetectionDebouncer = new Debouncer(Constants.Elevator.VERTICAL_CURRENT_SPIKE_DEBOUNCE_MIN_SECONDS, DebounceType.kRising);
    private ElevatorVerticalPosition desiredVerticalPosition;
    private MotorRunMode motorRunMode = MotorRunMode.POSITION_CONTROL;
    //Track count and duration of times output current exceeds 20 Amps
    private boolean trackCurrentThreshold;
    private ThresholdValueTimeTracker outputCurrentThresholdTracker = new ThresholdValueTimeTracker(20.0);
    private final Alert debugAlert = new Alert("Elevator current tracking debug", AlertType.kInfo);

    private final BooleanSupplier atGoalSupplier;
    private final Trigger readyToEjectCoral;

    private BooleanPublisher atGoalPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/RBR/Elevator/AtGoal")
            .publish();
    private StringPublisher goalVerticalPositionPublisher = NetworkTableInstance.getDefault()
            .getStringTopic("/RBR/Elevator/Height/Target").publish();
    private DoublePublisher currentHeightPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("/RBR/Elevator/Height/Current").publish();
    private DoublePublisher goalHeightPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("/RBR/Elevator/Height/Goal").publish();
    private DoublePublisher goalHeightUpperPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("/RBR/Elevator/Height/Goal/Upper").publish();
    private DoublePublisher goalHeightLowerPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("/RBR/Elevator/Height/Goal/Lower").publish();

    private DoublePublisher leftMotorEncoderPositionPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("/RBR/Elevator/LeftMotor/Position").publish();
    private DoublePublisher leftMotorOutputCurrentPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("/RBR/Elevator/LeftMotor/Current").publish();
    private DoublePublisher leftMotorVelocityPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("/RBR/Elevator/LeftMotor/Velocity").publish();

    // This gearbox represents a gearbox containing 2 NEO motors.
    private DCMotor verticalElevatorSimGearbox = DCMotor.getNEO(2);
    private SparkMaxSim verticalMotorsSim;
    private SparkRelativeEncoderSim verticalEncoderSim;
    private ElevatorSim verticalElevatorSim;

    private final ProfiledPIDController verticalProfiledPidController = new ProfiledPIDController(
            Constants.Elevator.kElevatorKp,
            Constants.Elevator.kElevatorKi,
            Constants.Elevator.kElevatorKd,
            new TrapezoidProfile.Constraints(3.5, 3.0));
            // new TrapezoidProfile.Constraints(2.45, 2.45));

    private Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private MechanismLigament2d m_elevatorMech2d;

    static {
        elevatorPositionToSetpointMeters = new EnumMap<>(ElevatorVerticalPosition.class);

        elevatorPositionToSetpointMeters.put(ElevatorVerticalPosition.L1, Constants.Elevator.POSITION_L1);
        elevatorPositionToSetpointMeters.put(ElevatorVerticalPosition.L2, Constants.Elevator.POSITION_L2);
        elevatorPositionToSetpointMeters.put(ElevatorVerticalPosition.HIGH_ALGAE, Constants.Elevator.POSITION_HIGH_ALGAE);
        elevatorPositionToSetpointMeters.put(ElevatorVerticalPosition.L3, Constants.Elevator.POSITION_L3);
        elevatorPositionToSetpointMeters.put(ElevatorVerticalPosition.L4, Constants.Elevator.POSITION_L4);
        elevatorPositionToSetpointMeters.put(ElevatorVerticalPosition.BARGE, Constants.Elevator.POSITION_BARGE);

        SmartDashboard.putNumber("elevatorKp", Constants.Elevator.kElevatorKp);
        SmartDashboard.putNumber("elevatorKi", Constants.Elevator.kElevatorKi);
        SmartDashboard.putNumber("elevatorKd", Constants.Elevator.kElevatorKd);
        SmartDashboard.putNumber("elevatorKs", Constants.Elevator.kElevatorKs);
    }

    public Elevator() {
        debugAlert.set(true);

        leftMotor = new SparkMax(Constants.CanID.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig leftMotorConfig = getVerticalMotorLeaderConfig(false, Constants.Elevator.kElevatorKp,
                Constants.Elevator.kElevatorKi, Constants.Elevator.kElevatorKd);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        leftEncoder = leftMotor.getEncoder();

        rightMotor = new SparkMax(Constants.CanID.ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig followerConfig = getVerticalMotorFollowerConfig();
        followerConfig.follow(leftMotor, true);
        rightMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightEncoder = rightMotor.getEncoder();

        // this assumes the elevator is at the bottom and tilted inwards (inside robot frame) when the robot is powered on
        desiredVerticalPosition = ElevatorVerticalPosition.L1;
        atGoal = true;
        resetVerticalElevatorEncoders();

        if (Robot.isSimulation()) {
            verticalElevatorSimGearbox = DCMotor.getNEO(2);
            verticalMotorsSim = new SparkMaxSim(leftMotor, verticalElevatorSimGearbox);
            verticalEncoderSim = verticalMotorsSim.getRelativeEncoderSim();
            verticalElevatorSim = new ElevatorSim(verticalElevatorSimGearbox, Constants.Elevator.kElevatorGearing,
                    Constants.Elevator.kCarriageMass,
                    Constants.Elevator.kElevatorDrumRadius, Constants.Elevator.kMinElevatorHeightMeters,
                    Constants.Elevator.kMaxElevatorHeightMeters,
                    true, 0, 0.01, 0.0);
            m_elevatorMech2d = m_mech2dRoot
                    .append(new MechanismLigament2d("Elevator", verticalElevatorSim.getPositionMeters(), 90));

            SmartDashboard.putData("Elevator Sim", m_mech2d);
        }

        verticalFeedForward = new ElevatorFeedforward(Constants.Elevator.kElevatorKs, Constants.Elevator.kElevatorKg,
                Constants.Elevator.kElevatorKv, Constants.Elevator.kElevatorKa);

        ksVerticalElevator = Constants.Elevator.kElevatorKs;

        atGoalSupplier = () -> atGoal;
        readyToEjectCoral = new Trigger(atGoalSupplier);
    }

    /** Advance the simulation. */
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        verticalElevatorSim.setInput(verticalMotorsSim.getAppliedOutput() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        verticalElevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        verticalEncoderSim.setPosition(verticalElevatorSim.getPositionMeters());
        verticalEncoderSim.setVelocity(verticalElevatorSim.getVelocityMetersPerSecond());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(verticalElevatorSim.getCurrentDrawAmps()));

        // Update elevator visualization with position
        m_elevatorMech2d.setLength(verticalEncoderSim.getPosition());
    }

    private void updateTelemetry() {
        leftMotorOutputCurrentAmps = leftMotor.getOutputCurrent();
        leftMotorEncoderPosition = leftEncoder.getPosition();
        leftMotorEncoderVelocity = leftEncoder.getVelocity();

        // if (trackCurrentThreshold) {
        //     outputCurrentThresholdTracker.addReading(leftMotorOutputCurrentAmps);
        //     debugAlert.setText(outputCurrentThresholdTracker.describeResult(outputCurrentThresholdTracker.getResult()));
        // }

        // publish state to NT
        leftMotorOutputCurrentPublisher.set(leftMotorOutputCurrentAmps);
        leftMotorEncoderPositionPublisher.set(leftMotorEncoderPosition);
        leftMotorVelocityPublisher.set(leftMotorEncoderVelocity);

        currentHeight = getHeight();

        stallDetected = stallDetectionDebouncer.calculate(leftMotorOutputCurrentAmps >= Constants.Elevator.VERTICAL_CURRENT_SPIKE_THRESHOLD);

        currentHeightPublisher.set(currentHeight);
        goalHeightPublisher.set(goalHeight);
        goalHeightUpperPublisher.set(goalHeightUpperBound);
        goalHeightLowerPublisher.set(goalHeightLowerBound);
        atGoalPublisher.set(atGoal);
        goalVerticalPositionPublisher.set(desiredVerticalPosition.name());

        if (currentHeight >= goalHeightLowerBound && currentHeight <= goalHeightUpperBound) {
            atGoal = true;
            if (desiredVerticalPosition == ElevatorVerticalPosition.L1) {
                resetVerticalElevatorEncoders();
            }
        } else {
            atGoal = false;
        }

        // this is for initial calculation of kS for ElevatorFeedForward
        double smartDashboardKs = SmartDashboard.getNumber("elevatorKs", Constants.Elevator.kElevatorKs);
        if (ksVerticalElevator != smartDashboardKs) {
            ksVerticalElevator = smartDashboardKs;
            DataLogManager.log("RBR: Elevator kS updated to: " + ksVerticalElevator);
            verticalFeedForward = new ElevatorFeedforward(ksVerticalElevator, Constants.Elevator.kElevatorKg,
                    Constants.Elevator.kElevatorKv, Constants.Elevator.kElevatorKa);
        }

        double smartDashboardKp = SmartDashboard.getNumber("elevatorKp", Constants.Elevator.kElevatorKp);
        double smartDashboardKi = SmartDashboard.getNumber("elevatorKi", Constants.Elevator.kElevatorKi);
        double smartDashboardKd = SmartDashboard.getNumber("elevatorKd", Constants.Elevator.kElevatorKd);

        if (kpVerticalElevator != smartDashboardKp) {
            kpVerticalElevator = smartDashboardKp;
            verticalProfiledPidController.setP(kpVerticalElevator);
            DataLogManager.log("RBR: Elevator kP updated to: " + kpVerticalElevator);
        }
        if (kiVerticalElevator != smartDashboardKi) {
            kiVerticalElevator = smartDashboardKi;
            verticalProfiledPidController.setI(kiVerticalElevator);
            DataLogManager.log("RBR: Elevator kI updated to: " + kiVerticalElevator);
        }
        if (kdVerticalElevator != smartDashboardKd) {
            kdVerticalElevator = smartDashboardKd;
            verticalProfiledPidController.setD(kdVerticalElevator);
            DataLogManager.log("RBR: Elevator kD updated to: " + kdVerticalElevator);
        }
    }

    @Override
    public void periodic() {
        //actively maintain position unless we're trying to reach L1 and already at the goal
        if (motorRunMode == MotorRunMode.POSITION_CONTROL) {
            if (!atGoal) {
                double pidOutput = verticalProfiledPidController.calculate(leftEncoder.getPosition());
                double feedforwardOutput = verticalFeedForward.calculate(verticalProfiledPidController.getSetpoint().velocity);
                leftMotor.setVoltage(pidOutput + feedforwardOutput);
            } 
            
            if (atGoal && desiredVerticalPosition == ElevatorVerticalPosition.L1) {
                leftMotor.stopMotor();
            }
        }

        // update state
        updateTelemetry();
    }

    public Trigger readyToEjectCoral() {
        return readyToEjectCoral;
    }

    public Command elevatorTestVerticalSetpointCommand() {
        return getSetVerticalGoalCommand(ElevatorVerticalPosition.L2);
    }

    public Command elevatorForceL1AndResetEncodersCommand() {
        return this.run(() -> {
            trackCurrentThreshold = true;
            desiredVerticalPosition = ElevatorVerticalPosition.L1;
            motorRunMode = MotorRunMode.DUTY_CYCLE;
            atGoal = false;
            runVerticalSpeed(-0.5);
        }).until(
            () -> stallDetected
        ).andThen(
            () -> {
                atGoal = true;
                trackCurrentThreshold = false;
                runVerticalSpeed(0.0);
                motorRunMode = MotorRunMode.POSITION_CONTROL;
            }
        ).andThen(Commands.waitSeconds(0.1))
        .andThen(
            () -> resetVerticalElevatorEncoders()
        );
    }

    public Command getSetVerticalGoalCommand(ElevatorVerticalPosition goalPosition) {
        return startEnd(
            () -> initializeTargetHeight(goalPosition),
            () -> {}
        );
    }

    public void initializeTargetHeight(ElevatorVerticalPosition goalPosition) {
        motorRunMode = MotorRunMode.POSITION_CONTROL;

        if (goalPosition != desiredVerticalPosition) {
            desiredVerticalPosition = goalPosition;
            atGoal = false;
        }

        //Reduce speed to 75% when the elevator is up
        switch (desiredVerticalPosition) {
            case L1 : 
                RobotContainer.setMaxSpeedFactor(Constants.Kinematics.INITIAL_DRIVE_MAX_SPEED_FACTOR);
                break;
            case L2 :
                // RobotContainer.setMaxSpeedFactor(0.75);
                RobotContainer.setMaxSpeedFactor(Constants.Kinematics.INITIAL_DRIVE_MAX_SPEED_FACTOR);
                break;
            default:
                RobotContainer.setMaxSpeedFactor(0.5);
                break;
        }

        double goalInMeters = elevatorPositionToSetpointMeters.get(goalPosition);
        goalHeight = goalInMeters;
        goalHeightUpperBound = goalHeight + Constants.Elevator.GOAL_DISTANCE_TOLERANCE;
        goalHeightLowerBound = goalHeight - Constants.Elevator.GOAL_DISTANCE_TOLERANCE;
        verticalProfiledPidController.setGoal(goalInMeters);
    }

    private SparkMaxConfig getVerticalMotorLeaderConfig(boolean invert, double kP, double kI, double kD) {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.inverted(invert).idleMode(IdleMode.kBrake);
        sparkMaxConfig.encoder.positionConversionFactor(Constants.Elevator.POSITION_CONVERSION)
                .velocityConversionFactor(Constants.Elevator.VELOCITY_CONVERSION);
        sparkMaxConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_ELEVATOR)
                .secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_ELEVATOR);

        return sparkMaxConfig;
    }

    private SparkMaxConfig getVerticalMotorFollowerConfig() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.encoder.positionConversionFactor(Constants.Elevator.POSITION_CONVERSION)
                .velocityConversionFactor(Constants.Elevator.VELOCITY_CONVERSION);
        sparkMaxConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_ELEVATOR)
                .secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_ELEVATOR);

        return sparkMaxConfig;
    }

    public Command resetEncodersCommand() {
        return runOnce(() -> resetVerticalElevatorEncoders());
    }

    /**
     * Get the height in meters.
     *
     * @return Height in meters
     */
    private double getHeight() {
        return leftEncoder.getPosition();
    }

    public Command runVerticalSpeedCommand(DoubleSupplier speedSupplier) {
        return this.run(() -> runVerticalSpeed(speedSupplier.getAsDouble()));
    }

    private void runVerticalSpeed(double speed) {
        leftMotor.set(speed);
    }

    public void resetVerticalElevatorEncoders() {
        verticalProfiledPidController.reset(0.0);
        verticalProfiledPidController.setGoal(0.0);
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
        atGoal = true;
    }

    public ElevatorVerticalPosition getDesiredVerticalPosition() {
        return desiredVerticalPosition;
    }

    public boolean isAtGoal() {
        return atGoal;
    }

    @Override
    public void close() throws Exception {
        m_mech2d.close();
    }
}

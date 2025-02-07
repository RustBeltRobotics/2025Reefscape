package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BasicUnits;
import frc.robot.util.Utilities;

/**
 * Models an individual wheel swerve module: drive motor, steer motor, encoders (for reading distance traveled, turn angle, steer velocity, etc.)
 */
public class SwerveModule extends SubsystemBase {

    // Drive PID Constants
    public static final double DRIVE_FEEDFORWARD_KS = 0.1759;  //from sysId - using Rotations as unit type, Simple as mechanism
    public static final double DRIVE_FEEDFORWARD_KV = 0.11294;  //from sysId
    public static final double DRIVE_FEEDFORWARD_KA = 0.0054209;  //from sysId
    public static final double DRIVE_P = 0.16367; //from sysId using CTRE phoenix 6 preset
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;

    // Steer PID Constants
    // For how to tune these values, see: https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/17
    //TODO: test with these values as well
    //  https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/blob/develop/src/main/java/com/swervedrivespecialties/swervelib/Mk4iSwerveModuleHelper.java#L40
    //  https://yagsl.gitbook.io/yagsl/configuring-yagsl/how-to-tune-pidf#starting-points
    public static final double STEER_P = 0.01;
    public static final double STEER_I = 0.0;
    public static final double STEER_D = 0.0;
    public static final double STEER_FF = 0.0;

    private final TalonFX driveMotor;
    private final SparkMax steerMotor;

    private SparkClosedLoopController steerPidController;
    private final RelativeEncoder steerEncoder;

    private final CANcoder absoluteSteerEncoder;

    public SwerveModule(int driveID, int steerID, int encoderID) {
        // Setup drive motor TalonFX
        driveMotor = new TalonFX(driveID);
        TalonFXConfiguration driveConfig = getDriveTalonFXConfiguration();
        driveMotor.getConfigurator().apply(driveConfig);
        driveMotor.setPosition(0);

        // Setup steer motor SparkMax
        SparkMaxConfig steerMotorSparkMaxConfig = getSteerSparkConfig();
        steerMotor = new SparkMax(steerID, MotorType.kBrushless);
        steerMotor.configure(steerMotorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Setup PID functionality for steer motors
        steerPidController = steerMotor.getClosedLoopController();

        // Setup steer motor relative encoder
        steerEncoder = steerMotor.getEncoder();

        // Setup steer motor absolute encoder
        absoluteSteerEncoder = new CANcoder(encoderID);

        resetEncoders(); // Zero encoders to ensure steer relative matches absolute
    }

    private TalonFXConfiguration getDriveTalonFXConfiguration() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        //Drive PIDF values
        Slot0Configs slot0 = new Slot0Configs();
        // slot0.kV = 0.10884; 
        // slot0.kA = 0.023145; 
        // slot0.kP = 0.07; 
        slot0.kS = DRIVE_FEEDFORWARD_KS; 
        slot0.kV = DRIVE_FEEDFORWARD_KV; 
        slot0.kA = DRIVE_FEEDFORWARD_KA;
        slot0.kP = DRIVE_P; 
        slot0.kI = DRIVE_I; 
        slot0.kD = DRIVE_D; 

        driveConfig.Slot0 = slot0;
        
        // Direction and neutral mode
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Ramp rates
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.0;
        
        // Gear ratio
        driveConfig.Feedback.SensorToMechanismRatio = 1.0; // 1:1 sensor to mechanism ratio (conversion factor is handled explicitly in code)

        // Current limits
        driveConfig.CurrentLimits.StatorCurrentLimit = 120; // 120A stator current limit
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Enable stator current limiting

        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = +120;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -120;
        driveConfig.TorqueCurrent.TorqueNeutralDeadband = 0.05; // 5% torque neutral deadband

        return driveConfig;
    }

    private SparkMaxConfig getSteerSparkConfig() {
        SparkMaxConfig steerMotorSparkMaxConfig = new SparkMaxConfig();
        steerMotorSparkMaxConfig.inverted(Constants.Kinematics.STEER_MOTOR_INVERTED).idleMode(IdleMode.kBrake);
        steerMotorSparkMaxConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_STEER).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_STEER);
        steerMotorSparkMaxConfig.encoder.positionConversionFactor(Constants.Kinematics.STEER_POSITION_CONVERSION).velocityConversionFactor(Constants.Kinematics.STEER_VELOCITY_CONVERSION);
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        steerMotorSparkMaxConfig.closedLoop.pidf(STEER_P, STEER_I, STEER_D, STEER_FF).iZone(0.0).outputRange(-1.0, 1.0)
            .positionWrappingEnabled(true).positionWrappingMaxInput(360.0).positionWrappingMinInput(0.0);

        return steerMotorSparkMaxConfig;
    }

    /** @return Drive position, meters, -inf to +inf */
    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * Constants.Kinematics.DRIVE_POSITION_CONVERSION;
    }

    /** @return Steer position, degrees, -inf to +inf */
    public double getSteerPosition() {
        return steerEncoder.getPosition();
    }

    /*
     * Get angle of steer motor in range -180 to 180 (useful for odometry publication to NT for viewing in AdvantageScope)
     */
    public double getSteerPositionConstrained() {
        double rawPosition = steerEncoder.getPosition();

        return rawPosition % 180.0;
    }

    /** @return Drive position, meters/second */
    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * Constants.Kinematics.DRIVE_VELOCITY_CONVERSION;
    }

    /** @return Steer position, degrees/second */
    public double getSteerVelocity() {
        return steerEncoder.getVelocity();
    }

    /** @return Absolute steer position, degrees, -inf to +inf */
    public double getAbsolutePosition() {
        return absoluteSteerEncoder.getAbsolutePosition().getValueAsDouble() * BasicUnits.DEGREES_PER_REVOLUTION;
    }

    /** @return Position of this swerve module based on Drive encoder (meters) and steer encoder (Rotation2d in radians) positions */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(Math.toRadians(getSteerPosition())));
    }

    /**
     * Resets the drive relative encoder to 0 and steer relative encoder to match the absolute encoder
     */
    public void resetEncoders() {
        driveMotor.setPosition(0.0);
        steerEncoder.setPosition(getAbsolutePosition());
    }

    /**
     * @return The module state (velocity, m/s, and steer angle, Rotation2d)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getSteerPosition()));
    }

    /**
     * @return The module state (velocity, m/s, and steer angle, Rotation2d - constrained to -pi to pi radians)
     */
    public SwerveModuleState getStateRotationConstrained() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getSteerPositionConstrained()));
    }

    /**
     * Sets the speed and angle of this swerve module
     * 
     * @param state the desired state (velocity in m/s, and steer angle as Rotation2d)
     */
    public void setState(SwerveModuleState state) {
        // If input is minimal, ignore input to avoid reseting steer angle to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stopModule();
            return;
        }
        
        Rotation2d currentAngle = Rotation2d.fromDegrees(getSteerPosition());
        // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#module-angle-optimization
        state.optimize(currentAngle);
        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother driving.
        state.cosineScale(currentAngle);
        final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
        double rotationsPerSecond = state.speedMetersPerSecond / Constants.Kinematics.DRIVE_VELOCITY_CONVERSION;
        driveMotor.setControl(velocityVoltage.withVelocity(rotationsPerSecond));
        setSteerAngle(state.angle.getDegrees());
    }

    /**
     * Locks the wheel at the provided angle
     * 
     * @param angle degrees
     */
    public void lockModule(int angle) {
        steerPidController.setReference(angle, SparkMax.ControlType.kPosition);
    }

    /** Set's the voltage to both motors to 0 */
    public void stopModule() {
        driveMotor.set(0.);
        steerMotor.set(0.);
    }

    /**
     * Set angle of steer motor - note resulting output will be optimized to minimize total spin distance
     * 
     * @param targetAngleInDegrees
     */
    public void setSteerAngle(double targetAngleInDegrees) {
        double currentSparkAngle = getSteerPosition();
        double sparkRelativeTargetAngle = Utilities.reboundValue(targetAngleInDegrees, currentSparkAngle);
        steerPidController.setReference(sparkRelativeTargetAngle, ControlType.kPosition);
    }

    /**
     * Set angle of steer motor - no optimizations will be applied to the target angle (useful for steer PID tuning)
     * 
     * @param targetAngleInDegrees
     */
    public void setSteerAngleAbsolute(double targetAngleInDegrees) {
        REVLibError pidResult = steerPidController.setReference(targetAngleInDegrees, ControlType.kPosition);
        Utilities.verifySparkMaxStatus(pidResult, steerMotor.getDeviceId(), "Swerve Steer Motor", "setPIDReferenceAbsolute");
    }
}
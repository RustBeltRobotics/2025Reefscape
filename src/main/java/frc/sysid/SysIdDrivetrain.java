package frc.sysid;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

/**
 * Drive subsystem for SysId characterization of swerve drive motors.
 */
public class SysIdDrivetrain extends SubsystemBase {
    
    private final SparkMax frontLeftDriveMotor;
    private final RelativeEncoder frontLeftDriveEncoder;
    private final SparkMax frontRightDriveMotor;
    private final RelativeEncoder frontRightDriveEncoder;
    private final SparkMax backRightDriveMotor;
    private final RelativeEncoder backRightDriveEncoder;
    private final SparkMax backLeftDriveMotor;
    private final RelativeEncoder backLeftDriveEncoder;
    
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    private final SysIdRoutine m_sysIdRoutine;

    public SysIdDrivetrain() {
        // Setup drive motor controllers
        SparkMaxConfig driveMotorSparkMaxConfig = new SparkMaxConfig();
        driveMotorSparkMaxConfig.inverted(Constants.Kinematics.DRIVE_MOTOR_INVERTED).idleMode(IdleMode.kBrake);
        driveMotorSparkMaxConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_DRIVE).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_DRIVE);
        driveMotorSparkMaxConfig.encoder.positionConversionFactor(Constants.Kinematics.DRIVE_POSITION_CONVERSION).velocityConversionFactor(Constants.Kinematics.DRIVE_VELOCITY_CONVERSION);

        frontLeftDriveMotor = new SparkMax(Constants.CanID.SWERVE_MODULE_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
        frontLeftDriveMotor.configure(driveMotorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        frontLeftDriveEncoder = frontLeftDriveMotor.getEncoder();

        frontRightDriveMotor = new SparkMax(Constants.CanID.SWERVE_MODULE_FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
        frontRightDriveMotor.configure(driveMotorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        frontRightDriveEncoder = frontRightDriveMotor.getEncoder();

        backRightDriveMotor = new SparkMax(Constants.CanID.SWERVE_MODULE_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
        backRightDriveMotor.configure(driveMotorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        backRightDriveEncoder = backRightDriveMotor.getEncoder();

        backLeftDriveMotor = new SparkMax(Constants.CanID.SWERVE_MODULE_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
        backLeftDriveMotor.configure(driveMotorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        backLeftDriveEncoder = backLeftDriveMotor.getEncoder();

        // Create a new SysId routine for characterizing the drive.
        m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motors.
                (Voltage volts) -> {
                    frontLeftDriveMotor.setVoltage(volts.in(Volts));
                    frontRightDriveMotor.setVoltage(volts.in(Volts));
                    backRightDriveMotor.setVoltage(volts.in(Volts));
                    backLeftDriveMotor.setVoltage(volts.in(Volts));
                },
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                    // Record a frame for the motors.
                    log.motor("drive-front-left")
                        // .voltage(m_appliedVoltage.mut_replace(frontLeftDriveMotor.get() * RobotController.getBatteryVoltage(), Volts))
                        .voltage(m_appliedVoltage.mut_replace(frontLeftDriveMotor.getAppliedOutput() * frontLeftDriveMotor.getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(frontLeftDriveEncoder.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(frontLeftDriveEncoder.getVelocity(), MetersPerSecond));
                    log.motor("drive-front-right")
                        // .voltage(m_appliedVoltage.mut_replace(frontRightDriveMotor.get() * RobotController.getBatteryVoltage(), Volts))
                        .voltage(m_appliedVoltage.mut_replace(frontLeftDriveMotor.getAppliedOutput() * frontLeftDriveMotor.getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(frontRightDriveEncoder.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(frontRightDriveEncoder.getVelocity(), MetersPerSecond));
                    log.motor("drive-back-right")
                        // .voltage(m_appliedVoltage.mut_replace(backRightDriveMotor.get() * RobotController.getBatteryVoltage(), Volts))
                        .voltage(m_appliedVoltage.mut_replace(frontLeftDriveMotor.getAppliedOutput() * frontLeftDriveMotor.getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(backRightDriveEncoder.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(backRightDriveEncoder.getVelocity(), MetersPerSecond));
                    log.motor("drive-back-left")
                        // .voltage(m_appliedVoltage.mut_replace(backLeftDriveMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .voltage(m_appliedVoltage.mut_replace(frontLeftDriveMotor.getAppliedOutput() * frontLeftDriveMotor.getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(backLeftDriveEncoder.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(backLeftDriveEncoder.getVelocity(), MetersPerSecond));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name ("drive")
                this)
        );
    }

      /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}

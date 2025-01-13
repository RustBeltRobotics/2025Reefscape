package frc.sysid;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

/**
 * Drive subsystem for SysId characterization of swerve drive motors.
 */
public class SysIdDrivetrain extends SubsystemBase {
    
    private final TalonFX frontLeftDriveMotor;
    private final TalonFX frontRightDriveMotor;
    private final TalonFX backRightDriveMotor;
    private final TalonFX backLeftDriveMotor;
    
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    private final SysIdRoutine m_sysIdRoutine;

    public SysIdDrivetrain() {
        // Setup drive motor controllers
        frontLeftDriveMotor = new TalonFX(Constants.CanID.SWERVE_MODULE_FRONT_LEFT_DRIVE_MOTOR);
        frontLeftDriveMotor.getConfigurator().apply(getTalonFXConfiguration());

        frontRightDriveMotor = new TalonFX(Constants.CanID.SWERVE_MODULE_FRONT_RIGHT_DRIVE_MOTOR);
        frontRightDriveMotor.getConfigurator().apply(getTalonFXConfiguration());

        backRightDriveMotor = new TalonFX(Constants.CanID.SWERVE_MODULE_BACK_RIGHT_DRIVE_MOTOR);
        backRightDriveMotor.getConfigurator().apply(getTalonFXConfiguration());

        backLeftDriveMotor = new TalonFX(Constants.CanID.SWERVE_MODULE_BACK_LEFT_DRIVE_MOTOR);
        backLeftDriveMotor.getConfigurator().apply(getTalonFXConfiguration());

        // Create a new SysId routine for characterizing the drive.
        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                            // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
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
                        .voltage(m_appliedVoltage.mut_replace(frontLeftDriveMotor.get() * frontLeftDriveMotor.getSupplyVoltage().getValueAsDouble(), Volts))
                        .linearPosition(m_distance.mut_replace(frontLeftDriveMotor.getPosition().getValueAsDouble() * Constants.Kinematics.DRIVE_POSITION_CONVERSION, Meters))
                        .linearVelocity(m_velocity.mut_replace(frontLeftDriveMotor.getVelocity().getValueAsDouble() * Constants.Kinematics.DRIVE_VELOCITY_CONVERSION, MetersPerSecond));
                    log.motor("drive-front-right")
                        .voltage(m_appliedVoltage.mut_replace(frontRightDriveMotor.get() * frontRightDriveMotor.getSupplyVoltage().getValueAsDouble(), Volts))
                        .linearPosition(m_distance.mut_replace(frontRightDriveMotor.getPosition().getValueAsDouble() * Constants.Kinematics.DRIVE_POSITION_CONVERSION, Meters))
                        .linearVelocity(m_velocity.mut_replace(frontRightDriveMotor.getVelocity().getValueAsDouble() * Constants.Kinematics.DRIVE_VELOCITY_CONVERSION, MetersPerSecond));
                    log.motor("drive-back-right")
                        .voltage(m_appliedVoltage.mut_replace(backRightDriveMotor.get() * backRightDriveMotor.getSupplyVoltage().getValueAsDouble(), Volts))
                        .linearPosition(m_distance.mut_replace(backRightDriveMotor.getPosition().getValueAsDouble() * Constants.Kinematics.DRIVE_POSITION_CONVERSION, Meters))
                        .linearVelocity(m_velocity.mut_replace(backRightDriveMotor.getVelocity().getValueAsDouble() * Constants.Kinematics.DRIVE_VELOCITY_CONVERSION, MetersPerSecond));
                    log.motor("drive-back-left")
                        .voltage(m_appliedVoltage.mut_replace(backLeftDriveMotor.get() * backLeftDriveMotor.getSupplyVoltage().getValueAsDouble(), Volts))
                        .linearPosition(m_distance.mut_replace(backLeftDriveMotor.getPosition().getValueAsDouble() * Constants.Kinematics.DRIVE_POSITION_CONVERSION, Meters))
                        .linearVelocity(m_velocity.mut_replace(backLeftDriveMotor.getVelocity().getValueAsDouble() * Constants.Kinematics.DRIVE_VELOCITY_CONVERSION, MetersPerSecond));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name ("drive")
                this)
        );
    }

    private TalonFXConfiguration getTalonFXConfiguration() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        // Direction and neutral mode
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  //TODO: verify/test this
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return driveConfig;
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

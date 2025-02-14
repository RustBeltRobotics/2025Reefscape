package frc.sysid;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;

public class SysIdVerticalElevator extends SysIdSubsystem {

    private final SparkMax leftMotor;
    private final RelativeEncoder leftEncoder;
    private final SparkMax rightMotor;
    private final RelativeEncoder rightEncoder;
    private final BooleanSupplier stopIfNearingMaxHeight;

    public SysIdVerticalElevator() {
        leftMotor = new SparkMax(Constants.CanID.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig leftConfig = getConfig();
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        leftEncoder = leftMotor.getEncoder();

        rightMotor = new SparkMax(Constants.CanID.ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig rightConfig = getConfig();
        rightConfig.follow(leftMotor, true);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightEncoder = rightMotor.getEncoder();

        stopIfNearingMaxHeight = () -> {
            double currentHeight = leftEncoder.getPosition();
            
            return currentHeight > (Constants.Elevator.kMaxElevatorHeightMeters - Units.inchesToMeters(2));
        };

        // Create a new SysId routine for characterizing the drive.
        m_sysIdRoutine = new SysIdRoutine(
             new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)s
                null
            ),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motors.
                (Voltage volts) -> {
                    leftMotor.setVoltage(volts.in(Volts));
                    rightMotor.setVoltage(volts.in(Volts));
                },
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                    // Record a frame for the motors.
                    log.motor("elevator-left")
                        .voltage(m_appliedVoltage.mut_replace(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(leftEncoder.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(leftEncoder.getVelocity(), MetersPerSecond));
                    log.motor("elevator-right")
                        .voltage(m_appliedVoltage.mut_replace(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(rightEncoder.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(rightEncoder.getVelocity(), MetersPerSecond));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name
                this)
        );
    }

    private Command resetEncodersCommand() {
        return runOnce(() -> {
            leftEncoder.setPosition(0);
            rightEncoder.setPosition(0);
        });
    }
    
    @Override
    public Command sysIdDynamic(Direction direction) {
        if (direction == Direction.kForward) {
            return Commands.sequence(resetEncodersCommand(), m_sysIdRoutine.dynamic(direction).until(stopIfNearingMaxHeight));
        } else {
            return m_sysIdRoutine.dynamic(direction).until(stopIfNearingMaxHeight);
        }
    }

    @Override
    public Command sysIdQuasistatic(Direction direction) {
        if (direction == Direction.kForward) {
            return Commands.sequence(resetEncodersCommand(), m_sysIdRoutine.quasistatic(direction).until(stopIfNearingMaxHeight));
        } else {
            return m_sysIdRoutine.quasistatic(direction).until(stopIfNearingMaxHeight);
        }
    }

    private SparkMaxConfig getConfig() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.inverted(false).idleMode(IdleMode.kBrake);
        sparkMaxConfig.encoder.positionConversionFactor(Constants.Elevator.POSITION_CONVERSION).velocityConversionFactor(Constants.Elevator.VELOCITY_CONVERSION);
        sparkMaxConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_ELEVATOR).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_ELEVATOR);

        return sparkMaxConfig;
    }

}

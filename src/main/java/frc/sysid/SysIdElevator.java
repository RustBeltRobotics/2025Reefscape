package frc.sysid;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class SysIdElevator extends SysIdSubsystem {

    private final SparkMax leftMotor;
    private final RelativeEncoder leftEncoder;
    private final SparkMax rightMotor;
    private final RelativeEncoder rightEncoder;

    public SysIdElevator() {
        leftMotor = new SparkMax(Constants.CanID.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
        //TODO: confirm if we should be inverting here or not
        SparkMaxConfig leftConfig = getConfig();
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        leftEncoder = leftMotor.getEncoder();

        rightMotor = new SparkMax(Constants.CanID.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
        //TODO: confirm if we should be inverting here or not
        SparkMaxConfig rightConfig = getConfig();
        rightConfig.follow(leftMotor, true);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightEncoder = rightMotor.getEncoder();

        // Create a new SysId routine for characterizing the drive.
        m_sysIdRoutine = new SysIdRoutine(
             new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                            // Log state with Phoenix SignalLogger class
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
                        .linearPosition(m_distance.mut_replace(Elevator.convertEncoderRotationsToDistance(Rotations.of(leftEncoder.getPosition())).in(Meters), Meters))
                        .linearVelocity(m_velocity.mut_replace(Elevator.convertEncoderRotationsToDistance(Rotations.of(leftEncoder.getVelocity())).per(Minute).in(MetersPerSecond), MetersPerSecond));
                    log.motor("elevator-right")
                        .voltage(m_appliedVoltage.mut_replace(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(Elevator.convertEncoderRotationsToDistance(Rotations.of(rightEncoder.getPosition())).in(Meters), Meters))
                        .linearVelocity(m_velocity.mut_replace(Elevator.convertEncoderRotationsToDistance(Rotations.of(rightEncoder.getVelocity())).per(Minute).in(MetersPerSecond), MetersPerSecond));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name
                this)
        );
    }

    private SparkMaxConfig getConfig() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.inverted(false).idleMode(IdleMode.kBrake);
        sparkMaxConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_ELEVATOR).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_ELEVATOR);

        return sparkMaxConfig;
    }

}

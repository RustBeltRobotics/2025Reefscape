package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private final SparkMax climberMotor;

    public Climber() {
        climberMotor = new SparkMax(Constants.CanID.CLIMBER_MOTOR, MotorType.kBrushed);
        SparkMaxConfig climbermotorConfig = new SparkMaxConfig();
        climbermotorConfig.inverted(false).idleMode(IdleMode.kBrake);
        climbermotorConfig.smartCurrentLimit(45).secondaryCurrentLimit(65); //TODO: confirm these current limits are good, and convert to finals in the Constants Class
        climberMotor.configure(climbermotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Sets the speeeeeeeeeeeeeeeed(BTW double is a noninteger )
     * @param speed how fast you want to go
     */
    public void climb(double speed) {
        climberMotor.set(speed);
    }

    /**
     * Stops the climber
     */
    public void stop() {
        climberMotor.set(0.);
    }
}

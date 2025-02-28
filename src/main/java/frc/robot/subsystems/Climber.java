package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private final SparkMax climberMotor;

    private DoublePublisher climberCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Climber/Current").publish();

    public Climber() {
        climberMotor = new SparkMax(Constants.CanID.CLIMBER_MOTOR, MotorType.kBrushed);
        SparkMaxConfig climbermotorConfig = getMotorConfig(IdleMode.kBrake);
        climberMotor.configure(climbermotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command changeMotorIdleModeCommand(IdleMode idleMode) {
        return runOnce(() -> climberMotor.configure(getMotorConfig(idleMode), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    private SparkMaxConfig getMotorConfig(IdleMode idleMode) {
        SparkMaxConfig climbermotorConfig = new SparkMaxConfig();
        climbermotorConfig.inverted(false).idleMode(idleMode);
        //TODO: confirm these current limits are good, and convert to finals in the Constants Class
        climbermotorConfig.smartCurrentLimit(65).secondaryCurrentLimit(Constants.CurrentLimit.Neo.SECONDARY); 

        return climbermotorConfig;
    }

    @Override
    public void periodic() {
        climberCurrentPublisher.set(climberMotor.getOutputCurrent());
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

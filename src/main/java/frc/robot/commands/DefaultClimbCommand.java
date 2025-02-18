package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climber;

public class DefaultClimbCommand extends Command {

    private final Climber climber;
    private final BooleanSupplier climbActiveSupplier;

    public DefaultClimbCommand(Climber climber, BooleanSupplier climbActiveSupplier) {
        this.climber = climber;
        this.climbActiveSupplier = climbActiveSupplier;

        // Command requires the climber subsystem
        addRequirements(climber);
    }

    /**
     * This method is run every 20 ms.
     */
    @Override
    public void execute() {
        if (!DriverStation.isAutonomous()){
            if (climbActiveSupplier.getAsBoolean()) {
                //if the climb button is being pressed, climb at full speed
                climber.climb(1.0);
            } else {
                climber.stop();
            }
        }
    }

    /**
     * When the climb method is interrupted, set motor speed to zero. 
     */
    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}

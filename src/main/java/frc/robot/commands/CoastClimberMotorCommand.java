package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class CoastClimberMotorCommand extends Command {

    private final Climber climber;

    public CoastClimberMotorCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

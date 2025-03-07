package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.LED;

public class DefaultLedCommand extends Command {

    private final LED ledSubsystem;

    public DefaultLedCommand(LED ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.setName("LED-Off");
        addRequirements(ledSubsystem);
    }

    /**
     * This method is run every 20 ms.
     */
    @Override
    public void execute() {
        ledSubsystem.changeColor(Color.kBlack);
    }
}
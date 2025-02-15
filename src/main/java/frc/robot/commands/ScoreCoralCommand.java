package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.model.ElevatorPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rejector;

/**
 * This command is used operate the elevator and rejector
 * to score coral at different levels.
 */
public class ScoreCoralCommand extends Command {
    private final Elevator elevator;
    private final Rejector rejector;
    private final ElevatorPosition level;
    private final double TOLERANCE = .03;
    private final int REJECTOR_RUN_CYCLES = 20;
    private final double REJECTOR_RUN_POWER = .5;

    public ScoreCoralCommand(Elevator elevator, Rejector rejector, ElevatorPosition level) {
        this.elevator = elevator;
        this.rejector = rejector;
        this.level = level;

        // Command requires elevator and rejector subsystem
        addRequirements(elevator);
        addRequirements(rejector);
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * Wait for the elvator to be in positoin
     */
    @Override
    public void execute() {
        switch(level) {
            case L1:
            if (elevator.getHeight() > Constants.Elevator.POSITION_L1 - TOLERANCE
                    &&  elevator.getHeight() < Constants.Elevator.POSITION_L1 + TOLERANCE) {
                double startPosition = rejector.getEncoderPosition();
                while (rejector.getEncoderPosition() > startPosition + REJECTOR_RUN_CYCLES) {
                    rejector.runAtPercentage(REJECTOR_RUN_POWER);
                }
                rejector.stopMotor();
                // TODO do we need to return elevator to rest?
                return;
            }
        
        
            case L2:
            if (elevator.getHeight() > Constants.Elevator.POSITION_L2 - TOLERANCE
                    &&  elevator.getHeight() < Constants.Elevator.POSITION_L2 + TOLERANCE) {
                double startPosition = rejector.getEncoderPosition();
                while (rejector.getEncoderPosition() > startPosition + REJECTOR_RUN_CYCLES) {
                    rejector.runAtPercentage(REJECTOR_RUN_POWER);
                }
                rejector.stopMotor();
                // TODO do we need to return elevator to rest?
                return;
            }
        
        
            case L3:
            if (elevator.getHeight() > Constants.Elevator.POSITION_L3 - TOLERANCE
                    &&  elevator.getHeight() < Constants.Elevator.POSITION_L3 + TOLERANCE) {
                double startPosition = rejector.getEncoderPosition();
                while (rejector.getEncoderPosition() > startPosition + REJECTOR_RUN_CYCLES) {
                    rejector.runAtPercentage(REJECTOR_RUN_POWER);
                }
                rejector.stopMotor();
                // TODO do we need to return elevator to rest?
                return;
            }
            case L4:
            if (elevator.getHeight() > Constants.Elevator.POSITION_L4 - TOLERANCE
            &&  elevator.getHeight() < Constants.Elevator.POSITION_L4 + TOLERANCE) {
        double startPosition = rejector.getEncoderPosition();
        while (rejector.getEncoderPosition() > startPosition + REJECTOR_RUN_CYCLES) {
            rejector.runAtPercentage(REJECTOR_RUN_POWER);
        }
        rejector.stopMotor();
        // TODO do we need to return elevator to rest?
        return;
    }
        }
    }

    /**
     * Get the elevator moving.
     */
    @Override
    public void initialize() {
        elevator.scoreCoral(level);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO do we need this?
    }
}

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class StopDrivetrainCommand extends Command {

    private final Drivetrain drivetrain;

    public StopDrivetrainCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void execute() {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }

    /** When the drive method is interupted, set all velocities to zero. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}

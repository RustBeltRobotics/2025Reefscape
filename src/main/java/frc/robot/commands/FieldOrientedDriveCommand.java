package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

/**
 * This command is used to drive the robot with a coordinate system that is
 * relative to the field, not the robot
 */
public class FieldOrientedDriveCommand extends Command {
    private final Drivetrain drivetrain;

    // DoubleSupplier objects need to be used, not double
    private final DoubleSupplier translationXSupplier;  //X+ is forward, X- is backward
    private final DoubleSupplier translationYSupplier;  //Y+ is strafing to the left, Y- is strafing to the right
    private final DoubleSupplier rotationSupplier;  //Positive is counter-clockwise, negative is clockwise

    public FieldOrientedDriveCommand(Drivetrain drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        // Command requires the drivetrain subsystem
        addRequirements(drivetrain);
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * Send the input x, y, and rotation velocities to the drivetrain's drive method as a ChassisSpeed object.
     */
    @Override
    public void execute() {
        //TODO: MJR - Monitor result of https://github.com/wpilibsuite/allwpilib/issues/7332
        // may want to change the sequence of discretize -> toSwerveModuleStates() -> desaturateWheelSpeeds to what is suggested in the linked issue 
        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationXSupplier.getAsDouble(),
                    translationYSupplier.getAsDouble(),
                    rotationSupplier.getAsDouble(),
                    drivetrain.getPoseRotation());
        drivetrain.drive(ChassisSpeeds.discretize(desiredSpeeds, TimedRobot.kDefaultPeriod));
    }

    /** When the drive method is interupted, set all velocities to zero. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }
}

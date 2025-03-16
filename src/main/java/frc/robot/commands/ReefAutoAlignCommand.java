package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.model.RejectorSide;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Rejector;

public class ReefAutoAlignCommand extends Command {

    //Distance from center of reef to coral arm on either side is approx 7" - if we've traveled 16", we definitely have gone too far
    private static final double TOTAL_DISTANCE_CUTOFF = Units.inchesToMeters(16.0);

    //Speed to move sideways in m/s (12 inches per second)
    private static final double STRAFE_VELOCITY = 0.3048;

    private final Drivetrain drivetrain;
    private final Trigger leftSensorTrigger;
    private final Trigger rightSensorTrigger;
    private final RejectorSide coralScoreSide;
    private Pose2d initialOdometryPose;
    private final Alert debugAlert = new Alert("ReefAutoAlign debug", AlertType.kInfo);

    public ReefAutoAlignCommand(Drivetrain drivetrain, Rejector rejector, RejectorSide coralScoreSide) {
        this.drivetrain = drivetrain;
        this.coralScoreSide = coralScoreSide;

        leftSensorTrigger = rejector.leftLaserSensorActive();
        rightSensorTrigger = rejector.rightLaserSensorActive();

        //we're only using the laser sensor triggers, so don't require the rejector as a subsystem requirement for command scheduling
        addRequirements(drivetrain);

        debugAlert.set(true);
    }

    @Override
    public void initialize() {
        initialOdometryPose = drivetrain.getSwerveOdometryPose();
    }

    /**
     * This method is run every 20 ms.
     */
    @Override
    public void execute() {
        //Y+ is strafing to the left, Y- is strafing to the right
        //Assuming robot is roughly in center of the reef, we need to strafe right to score left rejector side and vice versa
        double strafeVelocity = coralScoreSide == RejectorSide.LEFT ? -STRAFE_VELOCITY : STRAFE_VELOCITY;
        drivetrain.drive(new ChassisSpeeds(0.0, strafeVelocity, 0.0));
    }

    @Override
    public boolean isFinished() {
        double distanceTraveled = PhotonUtils.getDistanceToPose(initialOdometryPose, drivetrain.getSwerveOdometryPose());
        boolean exceededDistanceThreshold = distanceTraveled >= TOTAL_DISTANCE_CUTOFF;
        boolean sensorTriggered = coralScoreSide == RejectorSide.LEFT ? leftSensorTrigger.getAsBoolean() : rightSensorTrigger.getAsBoolean();

        if (exceededDistanceThreshold) {
            String logMsg = "ReefAutoAlign finished: exceeded distance threshold!";
            debugAlert.setText(logMsg);
            DataLogManager.log("RBR: " + logMsg);
        }
        if (sensorTriggered) {
            String logMsg = "ReefAutoAlign finished: " + coralScoreSide.name() + " laser sensor triggered!";
            debugAlert.setText(logMsg);
            DataLogManager.log("RBR: " + logMsg);
        }

        return exceededDistanceThreshold || sensorTriggered;
    }

    @Override
    public void end(boolean interrupted) {
        //stop drivetrain movement
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }
}

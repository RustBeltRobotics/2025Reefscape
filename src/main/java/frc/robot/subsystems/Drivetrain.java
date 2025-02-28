package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.hardware.MultiGyro;
import frc.robot.util.CollisionDetector;
import frc.robot.vision.VisionPoseEstimationResult;

/**
	- Contains majority of logic for driving the robot / controls the 4 SwerveModule instances (one for each wheel)
	- Publishes robot pose and swerve module state data to network tables (for use by AdvantageScope / PathPlanner)
	- Configures Auto chooser for selecting PathPlanner paths for Autonomous mode
	- Reads Gyro readings for robot angle (yaw) and angular velocity (using CTRE Pigeon2)
	- Updates robot odometry (estimation of location based on sensors) from swerve drive module states and vision system AprilTag readings (Limelight)
	- Handles special drive modes from driver controller (evasion and wheel locking)
 */
public class Drivetrain extends SubsystemBase {

    private final MultiGyro gyro;

    private final CollisionDetector collisionDetector;

    private VisionSystem visionSystem;  //set this externally if you want to use vision measurements for odometry

    /**
     * For user to reset zero for "forward" on the robot while maintaining absolute
     * field zero for odometry
     */

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    // The speed of the robot in x and y translational velocities and rotational velocity
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private double pitch;
    private double roll;
    private double yaw;

    private boolean pitchTipDetected = false;
    private boolean rollTipDetected = false;
    private boolean wheelsLockedX = false; // Boolean statement to control locking the wheels in an X-position

    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator poseEstimator;

    private SwerveModuleState[] swerveModuleStates = { new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };

    //Shuffleboard
    private static GenericEntry gyroEntry = Constants.Shuffleboard.COMPETITION_TAB.add("Gryoscope Angle", 0)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(0, 1)
            .withSize(3, 4)
            .getEntry();

    //https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html#networktables-organization
    // networktables publisher for advantagescope swerve visualization
    private StructArrayPublisher<SwerveModuleState> swerveStatePublisherMeasured = NetworkTableInstance.getDefault().getStructArrayTopic("/RBR/SwerveStates/Measured", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> swerveStatePublisherSetpoint = NetworkTableInstance.getDefault().getStructArrayTopic("/RBR/SwerveStates/Setpoint", SwerveModuleState.struct).publish();

    // networktables publisher for advantagescope 2d pose visualization
    private StructPublisher<Pose2d> poseEstimatePublisher = NetworkTableInstance.getDefault().getStructTopic("/RBR/PoseEstimated", Pose2d.struct).publish();
    private StructPublisher<Pose2d> visionResetPoseEstimatePublisher = NetworkTableInstance.getDefault().getStructTopic("/RBR/VisionResetPoseEstimated", Pose2d.struct).publish();
    private StructPublisher<Pose2d> poseSwerveOdometryPublisher = NetworkTableInstance.getDefault().getStructTopic("/RBR/PoseSwerveOdometry", Pose2d.struct).publish();

    // networktables publisher for advantagescope chassis speed visualization
    private StructPublisher<ChassisSpeeds> chassisSpeedPublisherSetpoint = NetworkTableInstance.getDefault().getStructTopic("/RBR/ChassisSpeed/Setpoint", ChassisSpeeds.struct).publish();

    private DoublePublisher frontLeftAbsoluteEncoderPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Rotation/Absolute/FL").publish();
    private DoublePublisher frontRightAbsoluteEncoderPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Rotation/Absolute/FR").publish();
    private DoublePublisher backLeftAbsoluteEncoderPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Rotation/Absolute/BL").publish();
    private DoublePublisher backRightAbsoluteEncoderPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Rotation/Absolute/BR").publish();

    private DoublePublisher frontLeftTorqueCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Current/Torque/FL").publish();
    private DoublePublisher frontRightTorqueCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Current/Torque/FR").publish();
    private DoublePublisher backLeftTorqueCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Current/Torque/BL").publish();
    private DoublePublisher backRightTorqueCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Current/Torque/BR").publish();

    private DoublePublisher frontLeftStatorCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Current/Stator/FL").publish();
    private DoublePublisher frontRightStatorCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Current/Stator/FR").publish();
    private DoublePublisher backLeftStatorCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Current/Stator/BL").publish();
    private DoublePublisher backRightStatorCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Current/Stator/BR").publish();

    private DoublePublisher pitchPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Gyro/Pitch").publish();
    private DoublePublisher rollPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Gyro/Roll").publish();
    private DoublePublisher yawPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Gyro/Yaw").publish();
    private DoublePublisher linearAccelerationXPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Gyro/Acceleration/X").publish();
    private DoublePublisher linearAccelerationYPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Gyro/Acceleration/Y").publish();

    public Drivetrain() {
        RobotConfig ppRobotConfig = Constants.PathPlanner.ROBOT_CONFIG;

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getEstimatedPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(Constants.PathPlanner.translation_P, Constants.PathPlanner.translation_I, Constants.PathPlanner.translation_D), // Translation PID constants
                        new PIDConstants(Constants.PathPlanner.rotation_P, Constants.PathPlanner.rotation_I, Constants.PathPlanner.rotation_D) // Rotation PID constants
                ),
                ppRobotConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        // Initialize all modules
        backRightModule = new SwerveModule(
            Constants.CanID.SWERVE_MODULE_BACK_RIGHT_DRIVE_MOTOR,
            Constants.CanID.SWERVE_MODULE_BACK_RIGHT_STEER_MOTOR,
            Constants.CanID.SWERVE_MODULE_BACK_RIGHT_STEER_ENCODER);

        backLeftModule = new SwerveModule(
            Constants.CanID.SWERVE_MODULE_BACK_LEFT_DRIVE_MOTOR,
            Constants.CanID.SWERVE_MODULE_BACK_LEFT_STEER_MOTOR,
            Constants.CanID.SWERVE_MODULE_BACK_LEFT_STEER_ENCODER);

        frontRightModule = new SwerveModule(
            Constants.CanID.SWERVE_MODULE_FRONT_RIGHT_DRIVE_MOTOR,
            Constants.CanID.SWERVE_MODULE_FRONT_RIGHT_STEER_MOTOR,
            Constants.CanID.SWERVE_MODULE_FRONT_RIGHT_STEER_ENCODER);

        frontLeftModule = new SwerveModule(
            Constants.CanID.SWERVE_MODULE_FRONT_LEFT_DRIVE_MOTOR,
            Constants.CanID.SWERVE_MODULE_FRONT_LEFT_STEER_MOTOR,
            Constants.CanID.SWERVE_MODULE_FRONT_LEFT_STEER_ENCODER);

        // Zero all relative encoders
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();

        gyro = new MultiGyro();

        collisionDetector = new CollisionDetector(gyro);
        
        Pose2d initialRobotPose = new Pose2d();
        Rotation2d initialRobotRotation = getGyroscopeRotation();
        SwerveModulePosition[] initialModulePositions = getSwerveModulePositions();
        swerveOdometry = new SwerveDriveOdometry(Constants.Kinematics.SWERVE_KINEMATICS, initialRobotRotation, initialModulePositions, initialRobotPose);

        // Create the poseEstimator with vectors to weight our vision measurements
        //See this post on how to tune the std deviation values: https://www.chiefdelphi.com/t/how-do-i-understand-standard-deviation-in-the-poseestimator-class/411492/10
        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Kinematics.SWERVE_KINEMATICS,
                initialRobotRotation, initialModulePositions, initialRobotPose,
                Constants.Kinematics.WHEEL_ODOMETRY_POSE_STANDARD_DEVIATIONS,
                Constants.Vision.DEFAULT_VISION_MEASUREMENT_STANDARD_DEVIATIONS);
        
        zeroPoseEstimatorAngle();
    }

    /**
     * Sets the poseEstimators position angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroPoseEstimatorAngle() {
        poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), new Rotation2d()));
    }

    public Command zeroPoseEstimatorAngleCommand() {
        return runOnce(() -> zeroPoseEstimatorAngle());
    }

    /**
     * Toggles whether or not the wheels are locked in X patterb. If they are, the wheels are
     * crossed into an X pattern and any other drive input has no effect.
     */
    public void toggleWheelsLockedX() {
        wheelsLockedX = !wheelsLockedX;
    }

    public double getGyroscopeAngle() {
        return gyro.getYaw();
    }

    // Returns the measurment of the gyroscope yaw. Used for field-relative drive
    public Rotation2d getGyroscopeRotation() {
        return gyro.getYawRotation2d();
    }

    /** @return Pitch in degrees, -180 to 180 */
    public double getPitch() {
        return gyro.getPitch();
    }

    /** @return Roll in degrees, -180 to 180 */
    public double getRoll() {
        return gyro.getRoll();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getPosition(), frontRightModule.getPosition(),
                backLeftModule.getPosition(), backRightModule.getPosition()
        };
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), pose);
        resetPoseEstimateUsingVision();
    }

    //will return true if pose was able to be reset using vision system
    public boolean resetPoseEstimateUsingVision() {
        if (Constants.Vision.VISION_ENABLED) {
            //This will force initial robot pose using vision system - overriding the initial pose set by PathPlanner auto
            Optional<VisionPoseEstimationResult> firstVisionPoseEstimationResult = visionSystem.getRobotPoseEstimationResults().stream().findFirst();
            if (firstVisionPoseEstimationResult.isPresent()) {
                Pose2d estimatedRobotPose = firstVisionPoseEstimationResult.get().getEstimatedRobotPose().estimatedPose.toPose2d();
                poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), estimatedRobotPose);
                visionResetPoseEstimatePublisher.set(estimatedRobotPose);

                return true;
            }
        }

        return false;
    }

    public Rotation2d getPoseRotation() {
        //this should always match 'actual' robot angle since the gyro is our source of truth
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public Trigger robotIsTipping() {
        return new Trigger(() -> pitchTipDetected || rollTipDetected);
    }

    public void updateOdometry() {
        Rotation2d currentRobotRotation = getGyroscopeRotation();
        yaw = currentRobotRotation.getDegrees();
        pitch = getPitch();
        roll = getRoll();

        pitchTipDetected = Math.abs(pitch) > Constants.Kinematics.TIP_THRESHOLD_DEGREES;
        rollTipDetected = Math.abs(roll) > Constants.Kinematics.TIP_THRESHOLD_DEGREES;

        SwerveModulePosition[] currentModulePositions = getSwerveModulePositions();
        swerveOdometry.update(currentRobotRotation, currentModulePositions);

        // Update position estimate using odometry from swerve states
        poseEstimator.update(currentRobotRotation, currentModulePositions);

        if (collisionDetector.isCollisionDetected() && collisionDetector.isStabilized()) {
            boolean validPostFromVision = resetPoseEstimateUsingVision();
            if (validPostFromVision) {
                collisionDetector.clearDetectedCollision();
            }
        }

        collisionDetector.checkForCollision();

        if (Constants.Vision.VISION_ENABLED) {
            List<VisionPoseEstimationResult> visionPoseEstimationResults = visionSystem.getRobotPoseEstimationResults();
            //TODO: review cases where we get multiple valid estimates back to determine if we should apply further filtering here to drop potential bad results
            for (VisionPoseEstimationResult visionPoseEstimationResult : visionPoseEstimationResults) {
                EstimatedRobotPose visionPoseEstimate = visionPoseEstimationResult.getEstimatedRobotPose();
                Pose2d estimatedRobotPoseFromVision = visionPoseEstimate.estimatedPose.toPose2d();
                poseEstimator.addVisionMeasurement(estimatedRobotPoseFromVision, visionPoseEstimate.timestampSeconds, visionPoseEstimationResult.getVisionMeasurementStdDevs());
            }
        }
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getSwerveOdometryPose() {
        return swerveOdometry.getPoseMeters();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return chassisSpeeds;
    }

    /**
     * Used to drive the robot with the provided ChassisSpeed object.
     * 
     * @param chassisSpeeds The translational and rotational velocities at which to
     *                      drive the robot.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * This method is used to command the individual module states based off the
     * ChassisSpeeds object
     */
    @Override
    public void periodic() {
        updateSwerveModuleStates();
        handleLockedStates();
        updateOdometry();
        updateTelemetry();
    }

    //update state of swerve modules based on current chassis speeds
    private void updateSwerveModuleStates() {
        swerveModuleStates = Constants.Kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    }

    private void handleLockedStates() {
        if (!wheelsLockedX) {
            // If we are not in wheel's locked mode, update swerve modules with new states based on chassis speeds
            updateSwerveModules(swerveModuleStates);
        } else if (wheelsLockedX) {
            // If we are in wheel's locked to X pattern mode, set the drive velocity to 0 so there is no
            // movment, and command the steer angle to either plus or minus 45 degrees to
            // form an X pattern.
            // See https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#rotation-conventions for angle conventions
            frontLeftModule.lockModule(45);
            frontRightModule.lockModule(-45);
            backLeftModule.lockModule(-45);
            backRightModule.lockModule(45);
        }
    }

    // Update the swerve modules with the new states
    private void updateSwerveModules(SwerveModuleState[] updatedModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(updatedModuleStates, Constants.Kinematics.MAX_SWERVE_MODULE_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.setState(updatedModuleStates[0]);
        frontRightModule.setState(updatedModuleStates[1]);
        backLeftModule.setState(updatedModuleStates[2]);
        backRightModule.setState(updatedModuleStates[3]);
    }

    private void updateTelemetry() {
        Pose2d estimatedPosition = getEstimatedPose();
        Pose2d swerveOdometryPosition = getSwerveOdometryPose();
        double currentRobotAngle = estimatedPosition.getRotation().getDegrees();
        // Publish gyro angle to shuffleboard
        gyroEntry.setDouble(currentRobotAngle);

        // Advantage scope things
        poseEstimatePublisher.set(estimatedPosition);
        poseSwerveOdometryPublisher.set(swerveOdometryPosition);
        
        chassisSpeedPublisherSetpoint.set(chassisSpeeds);

        swerveStatePublisherMeasured.set(new SwerveModuleState[] { 
            frontLeftModule.getState(), frontRightModule.getState(), backLeftModule.getState(), backRightModule.getState()
        });
        swerveStatePublisherSetpoint.set(swerveModuleStates);

        frontLeftAbsoluteEncoderPublisher.set(frontLeftModule.getAbsolutePosition());
        frontRightAbsoluteEncoderPublisher.set(frontRightModule.getAbsolutePosition());
        backLeftAbsoluteEncoderPublisher.set(backLeftModule.getAbsolutePosition());
        backRightAbsoluteEncoderPublisher.set(backRightModule.getAbsolutePosition());

        frontLeftTorqueCurrentPublisher.set(frontLeftModule.getDriveTorqueCurrent());
        frontRightTorqueCurrentPublisher.set(frontRightModule.getDriveTorqueCurrent());
        backLeftTorqueCurrentPublisher.set(backLeftModule.getDriveTorqueCurrent());
        backRightTorqueCurrentPublisher.set(backRightModule.getDriveTorqueCurrent());

        frontLeftStatorCurrentPublisher.set(frontLeftModule.getStatorCurrent());
        frontRightStatorCurrentPublisher.set(frontRightModule.getStatorCurrent());
        backLeftStatorCurrentPublisher.set(backLeftModule.getStatorCurrent());
        backRightStatorCurrentPublisher.set(backRightModule.getStatorCurrent());

        pitchPublisher.set(pitch);
        rollPublisher.set(roll);
        yawPublisher.set(yaw);
        linearAccelerationXPublisher.set(collisionDetector.getCurrentLinearAccelerationX());
        linearAccelerationYPublisher.set(collisionDetector.getCurrentLinearAccelerationY());
    }

    public void setVisionSystem(VisionSystem visionSystem) {
        this.visionSystem = visionSystem;
    }
    
}

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CollisionDetector;
import frc.robot.vision.VisionPoseEstimationResult;

/**
	- Contains majority of logic for driving the robot / controls the 4 SwerveModule instance (one for each wheel)
	- Publishes robot pose and swerve module state data to network tables (for use by AdvantageScope / PathPlanner)
	- Configures Auto chooser for selecting PathPlanner paths for Autonomous mode
	- Reads Gyro readings for robot angle (yaw) and angular velocity (using CTRE Pigeon2)
	- Updates robot odometry (estimation of location based on sensors) from swerve drive module states and vision system AprilTag readings (Limelight)
	- Handles special drive modes from driver controller (evasion and wheel locking)
 */
public class Drivetrain extends SubsystemBase {

    // Pigeon2 connected over CAN
    private final Pigeon2 gyro;

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
    StructArrayPublisher<SwerveModuleState> swerveStatePublisherMeasured = NetworkTableInstance.getDefault().getStructArrayTopic("/RBR/SwerveStates/Measured", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> swerveStatePublisherSetpoint = NetworkTableInstance.getDefault().getStructArrayTopic("/RBR/SwerveStates/Setpoint", SwerveModuleState.struct).publish();

    // networktables publisher for advantagescope 2d pose visualization
    StructPublisher<Pose2d> poseEstimatePublisher = NetworkTableInstance.getDefault().getStructTopic("/RBR/PoseEstimated", Pose2d.struct).publish();
    StructPublisher<Pose2d> visionResetPoseEstimatePublisher = NetworkTableInstance.getDefault().getStructTopic("/RBR/VisionResetPoseEstimated", Pose2d.struct).publish();
    StructPublisher<Pose2d> poseSwerveOdometryPublisher = NetworkTableInstance.getDefault().getStructTopic("/RBR/PoseSwerveOdometry", Pose2d.struct).publish();

    // networktables publisher for advantagescope chassis speed visualization
    // StructPublisher<ChassisSpeeds> chassisSpeedPublisherMeasured = NetworkTableInstance.getDefault().getStructTopic("/RBR/ChassisSpeed/Measured", ChassisSpeeds.struct).publish();
    //Navx velicity data is too inaccurate to make this useful
    StructPublisher<ChassisSpeeds> chassisSpeedPublisherSetpoint = NetworkTableInstance.getDefault().getStructTopic("/RBR/ChassisSpeed/Setpoint", ChassisSpeeds.struct).publish();

    DoublePublisher frontLeftAbsoluteEncoderPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Rotation/Absolute/FL").publish();
    DoublePublisher frontRightAbsoluteEncoderPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Rotation/Absolute/FR").publish();
    DoublePublisher backLeftAbsoluteEncoderPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Rotation/Absolute/BL").publish();
    DoublePublisher backRightAbsoluteEncoderPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Swerve/Rotation/Absolute/BR").publish();

    DoublePublisher linearAccelerationXPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Gyro/Acceleration/X").publish();
    DoublePublisher linearAccelerationYPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Gyro/Acceleration/Y").publish();

    public Drivetrain() {
        RobotConfig ppRobotConfig = null;

        try{
            //TODO: Move this to Constants.PathPlanner once values are solidified
            ppRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

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

        // Initialize and zero gyro
        gyro = new Pigeon2(Constants.CanID.PIGEON_GYRO);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

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
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        
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
        return gyro.getYaw().getValueAsDouble();
    }

    // Returns the measurment of the gyroscope yaw. Used for field-relative drive
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getGyroscopeAngle());
    }

    /** @return Pitch in degrees, -180 to 180 */
    public double getPitch() {
        return gyro.getPitch().getValueAsDouble();
    }

    /** @return Roll in degrees, -180 to 180 */
    public double getRoll() {
        return gyro.getRoll().getValueAsDouble();
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
        if (Constants.Vision.VISION_ENABLED && visionSystem != null) {
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

    public void updateOdometry() {
        Rotation2d currentRobotRotation = getGyroscopeRotation();
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

        if (Constants.Vision.VISION_ENABLED && visionSystem != null) {
            List<VisionPoseEstimationResult> visionPoseEstimationResults = visionSystem.getRobotPoseEstimationResults();
            //TODO: review cases where we get multiple valid estimates back to determine if we should apply further filtering here to drop potential bad results
            for (VisionPoseEstimationResult visionPoseEstimationResult : visionPoseEstimationResults) {
                EstimatedRobotPose visionPoseEstimate = visionPoseEstimationResult.getEstimatedRobotPose();
                Pose2d estimatedRobotPoseFromVision = visionPoseEstimate.estimatedPose.toPose2d();
                //TODO: define this method / test and/or use a constant here for the standard deviation
                // poseEstimator.setVisionMeasurementStdDevs(visionSystem.getVisionMeasurementStandardDeviation(visionPoseEstimate));
                poseEstimator.addVisionMeasurement(estimatedRobotPoseFromVision, visionPoseEstimate.timestampSeconds);
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
        //TODO: MJR - consider adding collision detection, eg. https://gist.githubusercontent.com/kauailabs/8c152fa14937b9cdf137/raw/900c99b23a1940e121ed1ae1abd589eb4050b5c1/CollisionDetection.java
        //set a state flag indicating collision occured - reset pose using vision then clear the flag if true
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
        linearAccelerationXPublisher.set(currentLinearAccelerationX);
        linearAccelerationYPublisher.set(currentLinearAccelerationY);
    }

    public void setVisionSystem(VisionSystem visionSystem) {
        this.visionSystem = visionSystem;
    }
    
}

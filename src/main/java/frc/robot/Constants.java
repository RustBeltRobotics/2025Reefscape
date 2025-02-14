// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class BasicUnits {
    public static final double SECONDS_PER_MINUTE = 60.0;
    public static final double DEGREES_PER_REVOLUTION = 360.0;
  }

  public static final class DriverStation {
    public static final int CONTROLLER_PORT_DRIVER = 0;
    public static final int CONTROLLER_PORT_OPERATOR = 1;
  }

  /**
   * CAN bus IDs
   */
  public static final class CanID {
    public static final int POWER_DISTRIBUTION = 1;
    public static final int PIGEON_GYRO = 10;
    public static final int ELEVATOR_LEFT_MOTOR = 14;
    public static final int ELEVATOR_RIGHT_MOTOR = 16;
    public static final int ELEVATOR_EXTEND_RETRACT_MOTOR = 12;
    public static final int REJECTOR_MOTOR = 19;
    public static final int SWERVE_MODULE_FRONT_LEFT_DRIVE_MOTOR = 6;
    public static final int SWERVE_MODULE_FRONT_LEFT_STEER_MOTOR = 13;
    public static final int SWERVE_MODULE_FRONT_LEFT_STEER_ENCODER = 2;
    public static final int SWERVE_MODULE_FRONT_RIGHT_DRIVE_MOTOR = 7;
    public static final int SWERVE_MODULE_FRONT_RIGHT_STEER_MOTOR = 17;
    public static final int SWERVE_MODULE_FRONT_RIGHT_STEER_ENCODER = 3;
    public static final int SWERVE_MODULE_BACK_RIGHT_DRIVE_MOTOR = 9;
    public static final int SWERVE_MODULE_BACK_RIGHT_STEER_MOTOR = 18;
    public static final int SWERVE_MODULE_BACK_RIGHT_STEER_ENCODER = 5;
    public static final int SWERVE_MODULE_BACK_LEFT_DRIVE_MOTOR = 8;
    public static final int SWERVE_MODULE_BACK_LEFT_STEER_MOTOR = 15;
    public static final int SWERVE_MODULE_BACK_LEFT_STEER_ENCODER = 4;
  }

  /**
   * Current limits - measure in Amps
   */
  public static final class CurrentLimit {
    public static final class SparkMax {
      public static final int SMART_STEER = 40;
      public static final int SMART_ELEVATOR = 40;
      public static final int SECONDARY_STEER = 80;
      public static final int SECONDARY_ELEVATOR = 80;
    }

    public static final class Neo {
      public static final double FREE_SPEED_RPMS = 5676.0;
      public static final int SMART = 60;
      public static final int SECONDARY = 80;
    }
  }

  /**
   * Robot physical constraints (max velocity, max angular velocity, SwerveDriveKinematics, etc.)
   */
  public static final class Kinematics {

    /* Robot mass in Kg. */
    public static final double MASS = Units.lbsToKilograms(106.0); //Note: this weight includes the battery (but no bumpers yet)

    /* Robot frame width in meters */
    public static final double WIDTH = Units.inchesToMeters(28.125);

    /* Robot width in meters WITH bumpers on */
    public static final double WIDTH_WITH_BUMPERS = Units.inchesToMeters(24.0); //TODO: update

    /* Robot frame length in meters */
    public static final double LENGTH = Units.inchesToMeters(28.125);

    /* Robot length in meters WITH bumpers on */
    public static final double LENGTH_WITH_BUMPERS = Units.inchesToMeters(29.5); //TODO: update

    /* Robot wheel diameter in meters */
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);

    /* Robot wheel circumference in meters */
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    /* Robot wheel radius in meters */
    public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;
    
    /**
     * The left-to-right distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.875);
    
    /**
     * The front-to-back distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.875);

    /** Distance from center of robot to center of swerve module  **/
    public static final double DRIVETRAIN_BASE_RADIUS = Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS, DRIVETRAIN_WHEELBASE_METERS) / 2.0;

    //SDS Mk4i L3 gear ratios - equivalent to 6.12:1 overall ratio
    public static final double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0); //Note: (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0) == 1.0/6.12 = 0.1634

    public static final double STEER_GEAR_RATIO = 1.0 / (150.0 / 7.0); //150/7:1 gear ratio

    //TODO: Figure out why true is needed for correct wheel direction, when in theory it should be false
    public static final boolean DRIVE_MOTOR_INVERTED = true; //should be false for Mk4i, true for Mk4 (tested on Eclipse robot)

    public static final boolean STEER_MOTOR_INVERTED = true; //should be true for Mk4i, false for Mk4 (tested on Eclipse robot)

    /** Conversion between motor rotations and drive meters */
    public static final double DRIVE_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * DRIVE_GEAR_RATIO;
        
    /** Conversion between motor rotations per second and drive meters per second */
    // public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / BasicUnits.SECONDS_PER_MINUTE;
    //Note: TalonFX reports velocity in RPS, not RPM - so no need to divide by 60
    //TODO: verify this exhibits the proper wheel velocity when testing drive
    public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION;

    /** Conversion between motor rotations and steer degrees */
    public static final double STEER_POSITION_CONVERSION = BasicUnits.DEGREES_PER_REVOLUTION * STEER_GEAR_RATIO;

    /** Conversion between motor rotations per minute and steer degrees per seconds */
    public static final double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / BasicUnits.SECONDS_PER_MINUTE;

    /**
     * The maximum linear velocity of a swerve module in meters per second. Calculate using https://www.reca.lc/drive
     */
    public static final double MAX_SWERVE_MODULE_VELOCITY_METERS_PER_SECOND = 5.48;  //TODO: compare to empirical measurement

    /**
     * The maximum angular velocity of the robot in radians per second. This is a
     * measure of how fast the robot can rotate in place.
     * 5.37 M/Sec / 0.41 M = 13.1 Rad / Sec = 750.57 degrees / sec
     */
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_SWERVE_MODULE_VELOCITY_METERS_PER_SECOND / DRIVETRAIN_BASE_RADIUS; //TODO: compare to empirical measurement (calc in comment seems high)

    /**
     * Used  to convert desired chassis velocity into individual swerve smodule states
     * See https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-drive-kinematics
     */
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),    //Front Left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),   //Front Right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),   //Back Left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)); //Back Right

    /* Change in linear acceleration greater than this value will trigger collision detected */
    public static final double COLLISION_THRESHOLD_DELTA_G = 0.5;
    /* Pose estimate should not be reset until after this long after collision */
    public static final long MICROSECONDS_SINCE_COLLISION_THRESHOLD = 250000;  //0.25 seconds

    public static final Matrix<N3, N1> WHEEL_ODOMETRY_POSE_STANDARD_DEVIATIONS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  }

  public static final class Elevator {
   
    //TODO: Need to test/tune these values
    public static final double kElevatorKp = 2.0892;  //this value is from sysId
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;

    //Feedforward values obtained via sysId:
    //https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A38%2C%22u%22%3A%22A%22%7D&efficiency=90&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A20%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A16%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A57.3%2C%22u%22%3A%22mm%22%7D&travelDistance=%7B%22s%22%3A27%2C%22u%22%3A%22in%22%7D   
    public static final double kElevatorKs = 0.18006; // volts (V)
    public static final double kElevatorKg = 0.18493; // volts (V)
    public static final double kElevatorKv = 5.3925; // volt per velocity (V/(m/s))
    public static final double kElevatorKa = 0.46421; // volt per acceleration (V/(m/s²))

    public static final double kElevatorGearing = 16.0;  //reduction
    public static final double kElevatorDrumRadius = Units.inchesToMeters(1.128);  //57.3 mm diameter
    public static final double kElevatorDrumCircumference = kElevatorDrumRadius * 2.0 * Math.PI;
    public static final double kCarriageMass = Units.lbsToKilograms(20.0);

    public static final double kMinElevatorHeightMeters = 0.0;
    // public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(27.0);  (27" * 2 for the cascade)
    public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(54.0); // ~ 1.3716 meterss

    public static final double kTiltGearing = 75.0;  //reduction

    public static final double POSITION_BOTTOM = 0.0;
     //TODO: test and confirm these values - these are just dummy values
    public static final double POSITION_L1 = Units.inchesToMeters(6.0);
    public static final double POSITION_L2 = Units.inchesToMeters(12.0);
    public static final double POSITION_L3 = Units.inchesToMeters(18.0);
    public static final double POSITION_L4 = Units.inchesToMeters(24.0);

    /** Conversion between motor rotations and elevator belt distance in meters */
    //Note: multiply by 2.0 to account for cascade design, which doubles the distance traveled
    public static final double POSITION_CONVERSION = kElevatorDrumCircumference * (1.0 / kElevatorGearing) * 2.0;
        
    /** Conversion between motor rotations per minute and elevator velocity in meters per second */
    public static final double VELOCITY_CONVERSION = POSITION_CONVERSION / BasicUnits.SECONDS_PER_MINUTE;
  }

  public static final class PathPlanner {
    public static final double rotation_P = 1.5;
    public static final double rotation_I = 0.0;
    public static final double rotation_D = 0.0;

    public static final double translation_P = 1.0;
    public static final double translation_I = 0.0;
    public static final double translation_D = 0.0;
  }

  public static final class Vision {
    public static final boolean VISION_ENABLED = true;
    public static final int APRIL_TAG_PIPELINE_INDEX = 0;
    public static final String ARDUCAM_MODEL = "OV9281";
    public static final double POSE_AMBIGUITY_CUTOFF = 0.2;  //https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html#ambiguity
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4.0;
    public static final double NOISY_DISTANCE_METERS = 2.5;  //distance beyond which vision measurements are noisy
    public static final double DISTANCE_CUTOFF = 4.0;  //Tag readings beyond this distance (in meters) will be considered invalid
    public static final double DISTANCE_WEIGHT = 7.0;
    public static final int TAG_PRESENCE_WEIGHT = 10;
    public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    /**
     * Standard deviations for vision measurements. Increase these numbers to trust your
     * vision measurements less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     * Note that the SwerveDrivePoseEstimator default is 0.9 for all values for vision measurements.
     * 
     * See also https://www.chiefdelphi.com/t/how-do-i-understand-standard-deviation-in-the-poseestimator-class/411492/10?u=mrokitka
     */
    public static final Matrix<N3, N1> DEFAULT_VISION_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder.fill(0.9, 0.9, 0.9);
    public static final Matrix<N3, N1> SINGLE_TAG_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(20));
    public static final double MULTI_TAG_XY_PER_TAG_STANDARD_DEVIATION = 0.6;
    public static final double MULTI_TAG_THETA_STANDARD_DEVIATION = Units.degreesToRadians(10);
    
    /**
     * Unique camera names, usable in PhotonCamera instances
     */
    public static final class CameraName {
      //Note: these names are set in hardware via https://docs.arducam.com/UVC-Camera/Serial-Number-Tool-Guide/
      public static final String FRONT_LEFT = "Arducam_OV9281_USB_Camera-2";
      public static final String FRONT_RIGHT = "Arducam_OV9281_USB_Camera-3";
      public static final String BACK_RIGHT = "Arducam_OV9281_USB_Camera-1";
      public static final String BACK_LEFT = "Arducam_OV9281_USB_Camera-4";
    }

    /**
     * Mounting position of the cameras on the Robot
     */
    public static final class CameraPose {
      private static final double CAM_XY_FROM_CENTER_OF_ROBOT = Units.inchesToMeters(10.0625);
      private static final double CAM_Z_FROM_FLOOR = Units.inchesToMeters(8.5);
      private static final double CAM_PITCH_ANGLE = -Units.degreesToRadians(25.0);
      //Note: these are robot to camera poses (position from center of robot to camera lens) - see also edu.wpi.first.math.ComputerVisionUtil.objectToRobotPose()
      //In transform3d - Translation3d values: x+ = forward, y+ = left, z+ = up, Rotation3d is rotation around the transform3d axes
      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      // https://vis-ro.web.app/robotics/eulerangles
      // https://www.chiefdelphi.com/t/photonvision-setup-specifically-the-robottocamera-transform/459246/2
      //Note: these values can be visualized in AdvantageScope if published over NetworkTables - https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/3D-FIELD.md
      //example - https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/apriltagvision/AprilTagVisionConstants.java#L30
      //x+, y+, z+, (0, -degrees, 0).rotateBy(0, 0, 45 degrees)
      public static final Transform3d FRONT_LEFT = new Transform3d(CAM_XY_FROM_CENTER_OF_ROBOT, CAM_XY_FROM_CENTER_OF_ROBOT, CAM_Z_FROM_FLOOR, 
        new Rotation3d(0, CAM_PITCH_ANGLE, 0).rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(45))));  //front left - photonvision1
      //x+, y-, z+, (0, -degrees, 0).rotateBy(0, 0, -45 degrees)
      public static final Transform3d FRONT_RIGHT = new Transform3d(CAM_XY_FROM_CENTER_OF_ROBOT, -CAM_XY_FROM_CENTER_OF_ROBOT, CAM_Z_FROM_FLOOR, 
        new Rotation3d(0, CAM_PITCH_ANGLE, 0).rotateBy(new Rotation3d(0, 0, -Units.degreesToRadians(45))));  //front right - photonvision2
      //x-, y-, z+, (0, -degrees, 0).rotateBy(0, 0, -135 degrees)
      public static final Transform3d BACK_RIGHT = new Transform3d(-CAM_XY_FROM_CENTER_OF_ROBOT, -CAM_XY_FROM_CENTER_OF_ROBOT, CAM_Z_FROM_FLOOR, 
        new Rotation3d(0, CAM_PITCH_ANGLE, 0).rotateBy(new Rotation3d(0, 0, -Units.degreesToRadians(135))));  //back right - photonvision1
      //x-, y+, z+, (0, -degrees, 0).rotateBy(0, 0, 135 degrees)
      public static final Transform3d BACK_LEFT = new Transform3d(-CAM_XY_FROM_CENTER_OF_ROBOT, CAM_XY_FROM_CENTER_OF_ROBOT, CAM_Z_FROM_FLOOR,
        new Rotation3d(0, CAM_PITCH_ANGLE, 0).rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(135))));  //back left - photonvision2
    }
  }

  public static final class Game {
    //Note: field layout values can be obtained by examining the AprilTagFieldLayout .json file for the game
    public static final double FIELD_LENGTH_METERS = 16.541;  //x in field drawings (from 2024 game - update for 2025 if necessary)
    public static final double FIELD_WIDTH_METERS = 8.211;  //y in field drawings (from 2024 game - update for 2025 if necessary)
    //These are buffers to accomodate for margin of error
    public static final double FIELD_POSE_XY_ERROR_MARGIN_METERS = Units.inchesToMeters(1.0);
    public static final double FIELD_POSE_THETA_ERROR_MARGIN_RADIANS = Units.degreesToRadians(2.0);
  }

  public static final class Shuffleboard {
    public static final ShuffleboardTab COMPETITION_TAB = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Competition");
    public static final ShuffleboardTab DIAG_TAB = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("diag");
  }
}

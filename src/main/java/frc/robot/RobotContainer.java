// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultClimbCommand;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.RobotOrientedDriveCommand;
import frc.robot.model.ElevatorVerticalPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorTiltMechanism;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Rejector;
import frc.robot.subsystems.VisionSystem;
import frc.robot.util.Utilities;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // For limiting maximum speed (1.0 = 100% - full speed)
  private static double MAX_SPEED_FACTOR = 1.0;

  private final CommandXboxController driverController = new CommandXboxController(Constants.DriverStation.CONTROLLER_PORT_DRIVER);
  private final CommandXboxController operatorController = new CommandXboxController(Constants.DriverStation.CONTROLLER_PORT_OPERATOR);
  private final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final ElevatorTiltMechanism elevatorTiltMechanism = new ElevatorTiltMechanism(elevator);
  private final Rejector rejector = new Rejector();
  private final Climber climber = new Climber();
  private final LED led = new LED();
  private final VisionSystem visionSystem;

  private final DoubleSupplier driverTranslationXSupplier;
  private final DoubleSupplier driverTranslationYSupplier;
  private final DoubleSupplier driverRotationSupplier;
    
  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Double> driveTrainSpeedChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    registerPathPlannerNamedCommands();
    
    autoChooser = AutoBuilder.buildAutoChooser();

    driveTrainSpeedChooser.setDefaultOption("100%", 1.0);
    driveTrainSpeedChooser.addOption("75%", 0.75);
    driveTrainSpeedChooser.addOption("50%", 0.5);
    driveTrainSpeedChooser.addOption("25%", 0.25);
    driveTrainSpeedChooser.onChange((newValue) -> RobotContainer.MAX_SPEED_FACTOR = newValue);
    Constants.Shuffleboard.COMPETITION_TAB.add("Drive Speed Selector", driveTrainSpeedChooser).withPosition(0, 2).withSize(2, 1);

    if (Constants.Vision.VISION_ENABLED) {
      visionSystem = new VisionSystem();
      drivetrain.setVisionSystem(visionSystem);
    } else {
      visionSystem = null;
    }

    driverTranslationXSupplier = () -> -Utilities.modifyDriverAxis(driverController.getLeftY(), 0.05) * Constants.Kinematics.MAX_SWERVE_MODULE_VELOCITY_METERS_PER_SECOND * MAX_SPEED_FACTOR;
    driverTranslationYSupplier = () -> -Utilities.modifyDriverAxis(driverController.getLeftX(), 0.05) * Constants.Kinematics.MAX_SWERVE_MODULE_VELOCITY_METERS_PER_SECOND * MAX_SPEED_FACTOR;
    driverRotationSupplier = () -> -Utilities.modifyDriverAxis(driverController.getRightX(), 0.05) * Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * MAX_SPEED_FACTOR;
    
    setDefaultCommands();
    configureBindings();
    configureAutos();

    // Speed up initial run of Pathplanner commands
    PathfindingCommand.warmupCommand().schedule();
  }

  private void registerPathPlannerNamedCommands() {
    //TODO: When removing PP named commands, paths or autos - make sure you clean the deploy folder on the Rio!
    // See https://www.chiefdelphi.com/t/pathplanner-autochooser-remembers-deleted-autos/455940/5
    Command tiltOutCommand = elevatorTiltMechanism.tiltOutCommand();
    Command tiltInCommand = elevatorTiltMechanism.tiltInCommand();
    Command elevatorL1Command = elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.L1).withTimeout(1.0);
    Command elevatorHighAlgaeCommand = elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.HIGH_ALGAE).withTimeout(2.5);
    Command elevatorL4Command = elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.L4).withTimeout(1.5);
    Command elevatorBargeCommand = elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.BARGE).withTimeout(1.5);
    Command doNothingCommand = Commands.none();
    NamedCommands.registerCommand("elevator-tilt-out", tiltOutCommand);
    NamedCommands.registerCommand("elevator-tilt-in", tiltInCommand);
    NamedCommands.registerCommand("coral-outtake", rejector.getOuttakeCommandWithSpeed(0.6).withTimeout(1.0));
    NamedCommands.registerCommand("algae-intake", rejector.getOuttakeCommand().withTimeout(2.5));
    NamedCommands.registerCommand("algae-outtake", rejector.getIntakeCommand().withTimeout(1.5));
    NamedCommands.registerCommand("elevator-l1", elevatorL1Command);
    NamedCommands.registerCommand("elevator-high-algae", elevatorHighAlgaeCommand);
    NamedCommands.registerCommand("elevator-l4", elevatorL4Command);
    NamedCommands.registerCommand("elevator-barge", elevatorBargeCommand);
    NamedCommands.registerCommand("grab-high-algae", Commands.race(rejector.getOuttakeCommand().withTimeout(2.5), elevatorHighAlgaeCommand));
    //when driving away from the reef after obtaining a high algae, lower the elevator
    // new EventTrigger("leaving-high-algae").onTrue(elevatorL1Command);

    //TODO: uncomment this after testing barge auto stop position can safety raise elevator without striking barge
    // new EventTrigger("approaching-barge").onTrue(elevatorBargeCommand);

    //TODO: comment this out after determining we can safeuly raise elevator to barge height and uncommenting above trigger
    new EventTrigger("approaching-barge").onTrue(doNothingCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //DRIVER bindings
    // Pressing A button sets forward direction to current robot heading
    driverController.a().onTrue(drivetrain.zeroPoseEstimatorAngleCommand().withName("ZeroPoseEstimatorAngle"));
    // Pressing B button will reset the pose estimator's current position using the first April Tag reading it sees
    driverController.b().onTrue(drivetrain.resetPoseUsingVisionCommand().withName("ResetPoseUsingVision"));

    //Drive in robot oriented mode while the left trigger is held
    RobotOrientedDriveCommand robotOrientedDriveCommand = new RobotOrientedDriveCommand(drivetrain, driverTranslationXSupplier, driverTranslationYSupplier, driverRotationSupplier);
    driverController.leftTrigger().whileTrue(robotOrientedDriveCommand);

    // Pressing X button rotates swerve wheels 45 degrees
    // TODO: remove this, it's only intended for swerve rotation PID tuning/testing!
    driverController.x().whileTrue(drivetrain.rotateWheels45DegreesCommand().withName("RotateWheels45"));
    driverController.x().onFalse(Commands.runOnce(() -> drivetrain.setWheelRotationPidTesting(false), drivetrain));

    driverController.povUp().whileTrue(climber.climbCommand());
    driverController.back().whileTrue(climber.descendCommand());
    // Pressing Down on the D-pad of driver controller will zero/reset vertical motor encoders of the elevator
    driverController.povDown().onTrue(elevator.resetEncodersCommand().withName("ResetElevatorEncoders"));

    //Test color LEDs when driver bumpers are held
    driverController.leftBumper().whileTrue(led.setLedColorCommand(Color.kRed).withName("LedColorRed"));
    driverController.rightBumper().whileTrue(led.setLedColorCommand(Color.kBlue).withName("LedColorBlue"));

    //TODO: on detection of right distance sensor, light up right side LED one color
    //TODO: on detection of left distance sensor, light up left side LED one color

    //OPERATOR bindings
    //Pressing A button moves elevator to L2 setpoint to score coral
    operatorController.a().onTrue(elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.L2).withName("ElevatorL2"));
    //Pressing B button moves elevator to L3 setpoint to score coral
    operatorController.b().onTrue(elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.L3).withName("ElevatorL3"));
    //Pressing X button moves elevator to L1 (bottom) setpoint to score coral
    operatorController.x().onTrue(elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.L1).withName("ElevatorL1"));
    //Pressing Y button moves elevator to L4 setpoint to score coral
    operatorController.y().onTrue(elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.L4).withName("ElevatorL4"));
    //Pressing R bumper moves elevator to barge net score position
    Command bargeHeightCommand = elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.BARGE).withName("ElevatorBarge");
    operatorController.rightBumper().onTrue(bargeHeightCommand);
    //Pressing L Bumper moves elevator to new setpoint for high algae on reef - (L3 level - 4 inches)
    operatorController.leftBumper().onTrue(elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.HIGH_ALGAE).withName("ElevatorHighAlgae"));

    //Pressing Down on the D-pad runs the avoid tipping command sequence: 
    // 1. Moves elevator to bottom position and tilts it in (if it is at a safe height to tilt in)
    // 2. Tilts the elevator back out a half second after completing #1 above
    Command avoidTippingCommand = Commands.parallel(elevator.elevatorBottomCommand(), elevatorTiltMechanism.tipAvoidanceTiltCommand())
      .andThen(Commands.waitSeconds(0.5))
      .andThen(elevatorTiltMechanism.tiltOutCommand())
      .withName("AvoidTipping");
    operatorController.povDown().onTrue(avoidTippingCommand);
    //Pressing Right Trigger performs outtake
    operatorController.rightTrigger().whileTrue(rejector.getOuttakeCommand().withName("Outtake"));
    //Pressing Left Trigger performs intake
    operatorController.leftTrigger().whileTrue(rejector.getIntakeCommand().withName("Intake"));
    //Pressing Left on the D-pad tilts elevator in to the robot frame
    operatorController.povLeft().onTrue(elevatorTiltMechanism.tiltInCommand().withName("TiltOut"));
    //Pressing Right on the D-pad tilts elevator out to the fully vertical position
    operatorController.povRight().onTrue(elevatorTiltMechanism.tiltOutCommand().withName("TiltIn"));
    //Pressing Start button moves the coral out slightly to make reef alignment easier
    operatorController.start().onTrue(rejector.getOuttakeCommand().withTimeout(0.05).withName("CoralEdge"));
    operatorController.back().whileTrue(climber.descendCommand());

    //TODO: Add explicit elevator tiltOut prior to raising elevator / after lowering elevator

    //Pressing Left or Right on the D-pad toggles between tilting the elevator out and in
    // Command toggleElevatorTiltCommand = elevatorTiltMechanism.toggleElevatorTiltCommand();
    // operatorController.povLeft().onTrue(toggleElevatorTiltCommand);
    // operatorController.povRight().onTrue(toggleElevatorTiltCommand);

    // automatically tilt the elevator inwards and bring to bottom position if the robot is tipping
    // drivetrain.robotIsTipping().onTrue(avoidTippingCommand);

    // Automatically eject the coral when the elevator reaches its target height
    //TODO: Re-enable this if we decide we want to auto-eject when elevator is at desired height
    // elevator.readyToEjectCoral().onTrue(rejector.scoreCoralCommand());
    
    //change climber motor from brake mode to coast mode 10 seconds after teleop ends
    //TODO: Test this
    // Trigger onTeleopInit = new Trigger(DriverStation::isTeleopEnabled);
    // onTeleopInit.onTrue(climber.changeMotorIdleModeCommand(IdleMode.kBrake)
    //   .andThen(Commands.waitSeconds(145), climber.changeMotorIdleModeCommand(IdleMode.kCoast))
    // );

    rejector.leftLaserSensorActive()
      .onTrue(Commands.runOnce(() -> rumbleControllers(true, true, false)))
      .onFalse(Commands.runOnce(() -> rumbleControllers(false, true, false)));

    rejector.rightLaserSensorActive()
      .onTrue(Commands.runOnce(() -> rumbleControllers(true, false, true)))
      .onFalse(Commands.runOnce(() -> rumbleControllers(false, false, true)));
  }

  private void setDefaultCommands() {
    //Default LED command removes all color output (sets to black)
    led.setDefaultCommand(new DefaultLedCommand(led));

    // DoubleSupplier translationXSupplier = () -> -Utilities.modifyAxisGeneric(driverController.getLeftY(), 1.0, 0.05) * Constants.Kinematics.MAX_SWERVE_MODULE_VELOCITY_METERS_PER_SECOND * MAX_SPEED_FACTOR;
    // DoubleSupplier translationYSupplier = () -> -Utilities.modifyAxisGeneric(driverController.getLeftX(), 1.0, 0.05) * Constants.Kinematics.MAX_SWERVE_MODULE_VELOCITY_METERS_PER_SECOND * MAX_SPEED_FACTOR;
    // DoubleSupplier rotationSupplier = () -> -Utilities.modifyAxisGeneric(driverController.getRightX(), 1.0, 0.05) * Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * MAX_SPEED_FACTOR;
    drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(drivetrain, driverTranslationXSupplier, driverTranslationYSupplier, driverRotationSupplier));

    //TODO: This seems to conflict with the pre-defined height bindings
    // DoubleSupplier elevatorVerticalSpeedSupplier = () -> -Utilities.modifyAxisGeneric(operatorController.getLeftY(), 1.0, 0.05);
    // elevator.setDefaultCommand(elevator.runVerticalSpeedCommand(elevatorVerticalSpeedSupplier));

    // DoubleSupplier elevatorTiltSpeedSupplier = () -> -Utilities.modifyAxisGeneric(operatorController.getLeftY(), 1.0, 0.05);
    // elevatorTiltMechanism.setDefaultCommand(elevatorTiltMechanism.elevatorTiltXBoxControllerCommand(elevatorTiltSpeedSupplier));
    // DoubleSupplier rejectorRotationSupplier = () -> -Utilities.modifyAxisGeneric(operatorController.getLeftX(), 1.0, 0.05);
    // rejector.setDefaultCommand(rejector.getRejectorOperatorCommand(rejectorRotationSupplier));

    BooleanSupplier climbActiveSupplier = () -> operatorController.povUp().getAsBoolean();
    climber.setDefaultCommand(new DefaultClimbCommand(climber, climbActiveSupplier));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command chosenAutoCommand = autoChooser.getSelected();

    return chosenAutoCommand;
    //Note: we can't do this, because the auto command is already a composition/sequence
    // (an exeption will be thrown)
    // Command elevatorTiltOutCommand = elevatorTiltMechanism.tiltOutCommand().withTimeout(1.0);
    // if (chosenAutoCommand != null) {
    //   //Before starting auto, ensure the elevator is tilted out into the vertical position
    //   return Commands.sequence(elevatorTiltOutCommand, chosenAutoCommand);
    // } else {
    //   return elevatorTiltOutCommand;
    // }
  }

  public void configureAutos() {
    Constants.Shuffleboard.COMPETITION_TAB.add("Auto Selector", autoChooser).withPosition(0, 0).withSize(2, 1);
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public void rumbleControllers(boolean rumble, boolean rumbleLeft, boolean rumbleRight) {
    double rumbleValue = rumble ? 0.25 : 0.0;
    XboxController driver = driverController.getHID();
    XboxController operator = operatorController.getHID();
    
    if (rumbleLeft) {
      driver.setRumble(RumbleType.kLeftRumble, rumbleValue);
      operator.setRumble(RumbleType.kLeftRumble, rumbleValue);
    }
    if (rumbleRight) {
      driver.setRumble(RumbleType.kRightRumble, rumbleValue);
      operator.setRumble(RumbleType.kRightRumble, rumbleValue);
    }
  }

  public static void setMaxSpeedFactor(double newSpeedFactor) {
    MAX_SPEED_FACTOR = newSpeedFactor;
  }
}

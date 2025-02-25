// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverStation;
import frc.robot.commands.DefaultClimbCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.model.ElevatorVerticalPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorTiltMechanism;
import frc.robot.subsystems.Rejector;
import frc.robot.subsystems.VisionSystem;
import frc.robot.util.Utilities;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

  private final CommandXboxController driverController = new CommandXboxController(DriverStation.CONTROLLER_PORT_DRIVER);
  private final CommandXboxController operatorController = new CommandXboxController(DriverStation.CONTROLLER_PORT_OPERATOR);
  private final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final ElevatorTiltMechanism elevatorTiltMechanism = new ElevatorTiltMechanism();
  private final Rejector rejector = new Rejector();
  private final Climber climber = new Climber();
  private final VisionSystem visionSystem;

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Double> driveTrainSpeedChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
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

    setDefaultCommands();
    // Configure the trigger bindings
    configureBindings();

    //TODO: register NamedCommands for pathplanner

    configureAutos();
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
    // Pressing A button sets forward direction to current robot heading
    driverController.a().onTrue(drivetrain.zeroPoseEstimatorAngleCommand());
    //Pressing X button moves elevator to L2 setpoint (just for testing / PID tuning)
    driverController.x().onTrue(elevator.elevatorTestVerticalSetpointCommand());
    //Pressing Y button stops elevator / moves it to bottom position
    driverController.y().onTrue(elevator.elevatorBottomCommand());

    //Pressing A button moves elevator to L2 setpoint to score coral
    operatorController.a().onTrue(elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.L2));
    //Pressing B button moves elevator to L3 setpoint to score coral
    operatorController.b().onTrue(elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.L3));
    //Pressing X button moves elevator to L1 (bottom) setpoint to score coral
    operatorController.x().onTrue(elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.L1));
    //Pressing Y button moves elevator to L4 setpoint to score coral
    operatorController.y().onTrue(elevator.getSetVerticalGoalCommand(ElevatorVerticalPosition.L4));
    //Pressing Down on the D-pad starts the avoid tipping command sequence (moves elevator to bottom position and tilts it in)
    Command avoidTippingCommand = Commands.parallel(elevator.elevatorBottomCommand(), elevatorTiltMechanism.tiltInCommand());
    operatorController.povDown().onTrue(avoidTippingCommand);
    operatorController.rightBumper().whileTrue(rejector.getOuttakeCommand());
    operatorController.leftBumper().whileTrue(rejector.getIntakeCommand());
    //Pressing Left or Right on the D-pad toggles between tilting the elevator out and in
    Command toggleElevatorTiltCommand = elevatorTiltMechanism.toggleElevatorTiltCommand();
    operatorController.povLeft().onTrue(toggleElevatorTiltCommand);
    operatorController.povRight().onTrue(toggleElevatorTiltCommand);

    // automatically tilt the elevator inwards and bring to bottom position if the robot is tipping
    drivetrain.robotIsTipping().onTrue(avoidTippingCommand);

    // Automatically eject the coral when the elevator reaches its target height
    //TODO: Re-enable this if we decide we want to auto-eject when elevator is at desired height
    // elevator.readyToEjectCoral().onTrue(rejector.scoreCoralCommand());
  }

  private void setDefaultCommands() {
    DoubleSupplier translationXSupplier = () -> -Utilities.modifyAxisGeneric(driverController.getLeftY(), 1.0, 0.05) * Constants.Kinematics.MAX_SWERVE_MODULE_VELOCITY_METERS_PER_SECOND * MAX_SPEED_FACTOR;
    DoubleSupplier translationYSupplier = () -> -Utilities.modifyAxisGeneric(driverController.getLeftX(), 1.0, 0.05) * Constants.Kinematics.MAX_SWERVE_MODULE_VELOCITY_METERS_PER_SECOND * MAX_SPEED_FACTOR;
    DoubleSupplier rotationSupplier = () -> -Utilities.modifyAxisGeneric(driverController.getRightX(), 1.0, 0.05) * Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * MAX_SPEED_FACTOR;
    drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(drivetrain, translationXSupplier, translationYSupplier, rotationSupplier));

    DoubleSupplier elevatorTiltSpeedSupplier = () -> -Utilities.modifyAxisGeneric(operatorController.getLeftY(), 1.0, 0.05);
    elevatorTiltMechanism.setDefaultCommand(elevatorTiltMechanism.elevatorTiltXBoxControllerCommand(elevatorTiltSpeedSupplier));
    DoubleSupplier rejectorRotationSupplier = () -> -Utilities.modifyAxisGeneric(operatorController.getLeftX(), 1.0, 0.05);
    rejector.setDefaultCommand(rejector.getRejectorOperatorCommand(rejectorRotationSupplier));

    BooleanSupplier climbActiveSupplier = () -> operatorController.povUp().getAsBoolean();
    climber.setDefaultCommand(new DefaultClimbCommand(climber, climbActiveSupplier));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void configureAutos() {
    Constants.Shuffleboard.COMPETITION_TAB.add("Auto Selector", autoChooser).withPosition(0, 0).withSize(2, 1);
  }

  //MJR: TODO: If we end up needing this, call it like something like this:
    // new InstantCommand(() -> rumbleControllers(true)).andThen(new WaitCommand(1.0)).finallyDo(() -> rumbleControllers(false));
  public void rumbleControllers(boolean rumble) {
    double rumbleValue = rumble ? 0.5 : 0.0;
    XboxController driver = driverController.getHID();
    XboxController operator = operatorController.getHID();
    
    driver.setRumble(RumbleType.kLeftRumble, rumbleValue);
    driver.setRumble(RumbleType.kRightRumble, rumbleValue);
    operator.setRumble(RumbleType.kLeftRumble, rumbleValue);
    operator.setRumble(RumbleType.kRightRumble, rumbleValue);
  }
}

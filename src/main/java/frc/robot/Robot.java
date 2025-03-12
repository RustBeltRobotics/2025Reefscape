// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.PowerManagement;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private final PowerManagement powerManagement;
  private GenericEntry timeEntry = Constants.Shuffleboard.COMPETITION_TAB.add("Time Left", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(3, 0)
            .withSize(4, 1)
            .withProperties(Map.of("min", 0, "max", 165))
            .getEntry();

  public Robot() {
    super();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    powerManagement = new PowerManagement();
    
    // Start datalogging of all networkTables entrys onto the RIO - see DataLogManager class javadoc - strongly recommended to attach a USB stick to the RIO for persisting log data
    // See https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html for more information
    DataLogManager.start();

    // Start datalogging of all DS data and joystick data onto the RIO
    DriverStation.startDataLog(DataLogManager.getLog());

    // Log Phoenix / CTRE device signals (this code is only necessary outside of competition)
    // SignalLogger.setPath("/media/sdb1/logs/");
    SignalLogger.start();

    // Log Rev device signals (see https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/REV-LOGGING.md)
    URCL.start();

    //Camera server is used to view USB cam as video stream on dashboard
    CameraServer.startAutomaticCapture();

    //allow access to photonvision coprocessor (orange pi) UIs when tethered to USB port on the rio
    // PortForwarder.add(5800, "photonvision1.local", 5800);
    // PortForwarder.add(5801, "photonvision2.local", 5800);

    //Send command scheduler data to smartdashboard
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //Note: code from robotInit() has been moved to the constructor to match upcoming Wpilib 2025 changes
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    timeEntry.setDouble(DriverStation.getMatchTime()); // Update the time left in shuffleboard
    powerManagement.updateTelemetry(); //check for brownouts and breaker faults on the PDH
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //turn off controller rumble when disabled
    robotContainer.rumbleControllers(false,true, true);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //If enabled in constants, use vision readings in auto to update odometry / pose estimate
    robotContainer.getDrivetrain().setShouldUseVision(Constants.Vision.VISION_ENABLED);
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    //Do not use vision / AprilTag readings to update odometry during tele-op
    robotContainer.getDrivetrain().setShouldUseVision(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

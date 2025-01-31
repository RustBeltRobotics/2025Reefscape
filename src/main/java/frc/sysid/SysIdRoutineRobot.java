package frc.sysid;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriverStation;

/**
 * Robot class for running SysId characterization routines on subsystems.
 */
public class SysIdRoutineRobot extends TimedRobot {

    private final CommandXboxController driverController = new CommandXboxController(DriverStation.CONTROLLER_PORT_DRIVER);
    private final SysIdDrivetrain drivetrain = new SysIdDrivetrain();
    private final SysIdElevator elevator = new SysIdElevator();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public SysIdRoutineRobot() {
        SysIdSubsystem subsystemUnderTest = elevator;
        
        // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
        // once.
        driverController
            .a()
            .and(driverController.rightBumper())
            .whileTrue(subsystemUnderTest.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driverController
            .b()
            .and(driverController.rightBumper())
            .whileTrue(subsystemUnderTest.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        driverController
            .x()
            .and(driverController.rightBumper())
            .whileTrue(subsystemUnderTest.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driverController
            .y()
            .and(driverController.rightBumper())
            .whileTrue(subsystemUnderTest.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {}

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

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
}

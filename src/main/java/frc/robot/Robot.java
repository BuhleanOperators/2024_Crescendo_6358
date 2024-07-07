// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.beltSubsystem;
import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.ledSubsystem;
import frc.robot.subsystems.pneumaticSubsystem;
import frc.robot.subsystems.shooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  //Initilize all the subsystems
  //! Only use subsystems by calling these
  private  RobotContainer m_robotContainer = new RobotContainer();
  public static final smartDashboard m_SmartDashboard = new smartDashboard();
  public static final beltSubsystem m_BeltSubsystem = new beltSubsystem();
  public static final intakeSubsystem m_IntakeSubsystem = new intakeSubsystem();
  public static final pneumaticSubsystem m_PneumaticSubsystem = new pneumaticSubsystem();
  public static final shooterSubsystem m_ShooterSubsytem = new shooterSubsystem();
  public static final driveTrain m_DriveTrain = new driveTrain();
  public static final ledSubsystem m_LedSubsystem = new ledSubsystem();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //Set up the gyro, alliance color, auto chooser and USB cammera
    m_DriveTrain.calibrateGyro();
    m_SmartDashboard.AllianceColor();
    m_SmartDashboard.AutoChooser();
    UsbCamera camera = CameraServer.startAutomaticCapture();    
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
    //Turn on the compressor, get the multiplier, gather robot data, get the auto selection, and check if the shooter is up to speed to score in the SPEAKER
    m_PneumaticSubsystem.compressorOn();
    m_SmartDashboard.multiplier();
    m_SmartDashboard.gatherData();
    m_SmartDashboard.AutoChooser();
    m_SmartDashboard.upToSpeed();
  }
  

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //Set the idle mode when the robot is disabled
    m_DriveTrain.setIdleMode(DriveConstants.idleModeDisabled);
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //Set the idle mode of the drive motors
    m_DriveTrain.setIdleMode(DriveConstants.idleModeAuto);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    //Reset the encoders and set the idle mode of the drive motors
    m_DriveTrain.resetEncoders();
    m_DriveTrain.setIdleMode(DriveConstants.idleModeTeleop);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

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

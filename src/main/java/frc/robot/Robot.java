// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Alert.AlertType;
import frc.robot.Constants.DriveConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot{
  private static final double canErrorTimeThresholdSec = 0.5;
  private static final double lowBatteryVoltageVolts = 11.8;
  private static final double lowBatteryDisabledTimeSec = 1.5;

  private double autoStart;
  private double teleStart;
  private boolean isAutoMessagePrinted = false;

  private final Timer disabledTimer = new Timer();
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();

  private static double teleElapsedTime = 0.0;

  private Command autoCommand;

  private  RobotContainer robotContainer;

  private final Alert canErrorAlert = 
    new Alert("CAN errors detected, robot may be uncontrolable.", AlertType.ERROR);
  private final Alert lowBatteryAlert = 
    new Alert("Battery voltage is very low, consider turning off the robot or replacing the battery.", AlertType.WARNING);

  public static Trigger createTeleopTimeTrigger(DoubleSupplier teleElapsedTime) {
    return new Trigger(
        () ->
          DriverStation.isFMSAttached()
            && DriverStation.isTeleopEnabled()
            && Robot.teleElapsedTime > teleElapsedTime.getAsDouble());
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes commited");
        break;
      case 1: 
        Logger.recordMetadata("GitDirty", "Uncommited changes");
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.currentMode){
      case REAL:
        //Running on real robot, log to USB stick ("/U/Logs") 
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
        
      case REPLAY:
        //Replaying a log, set up replay source
        setUseTiming(false); //Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }

    Logger.start(); 

    //Log active Commands
    Map<String, Integer> commandsCount = new HashMap<>();
    BiConsumer<Command, Boolean> logComandsFunction = 
      (Command command, Boolean active) -> {
        String name = command.getName();
        int count = commandsCount.getOrDefault(name, 0) + (active ? 1 : -1);
        commandsCount.put(name, count);
        Logger.recordOutput(
          "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
        Logger.recordOutput("CommandsAll/" + name, count > 0);
      };
    CommandScheduler.getInstance()
      .onCommandInitialize(
        (Command command) -> {
          logComandsFunction.accept(command, true);
          });
    CommandScheduler.getInstance()
        .onCommandFinish(
          (Command command) -> {
            logComandsFunction.accept(command, false);
          });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
          (Command comand) -> {
            logComandsFunction.accept(comand, false);
          });

    //Reset alert timers
    disabledTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    canInitialErrorTimer.restart();

    RobotController.setBrownoutVoltage(6.0);

    robotContainer = new RobotContainer();
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

    //Print auto message
    if (autoCommand != null){
      if (!autoCommand.isScheduled() && !isAutoMessagePrinted){
        if (DriverStation.isAutonomousEnabled()){
          System.out.printf("*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }else{
          System.out.printf("*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        isAutoMessagePrinted = true;
      }
    }

    robotContainer.checkControllers();
    robotContainer.updateDashboardOutputs();

    //Check CAN status
    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0){
      canErrorTimer.reset();
    }
    canErrorAlert.set(
      !canErrorTimer .hasElapsed(canErrorTimeThresholdSec) && !canInitialErrorTimer.hasElapsed(canErrorTimeThresholdSec));
    
    //Low battery alert
    if (DriverStation.isEnabled()){
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltageVolts 
      && disabledTimer.hasElapsed(lowBatteryDisabledTimeSec)){
        lowBatteryAlert.set(true);
      }
  }
  

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //Set the idle mode when the robot is disabled
    robotContainer.drive.setIdleMode(DriveConstants.idleModeDisabled);
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoStart = Timer.getFPGATimestamp();
    isAutoMessagePrinted = false;
    //Set the idle mode of the drive motors
    robotContainer.drive.setIdleMode(DriveConstants.idleModeAuto);

    autoCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    teleStart = Timer.getFPGATimestamp();
    //Set the idle mode of the drive motors
    robotContainer.drive.setIdleMode(DriveConstants.idleModeTeleop);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    robotContainer.pneumatics.climbDown();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    teleElapsedTime = Timer.getFPGATimestamp() - teleStart;
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

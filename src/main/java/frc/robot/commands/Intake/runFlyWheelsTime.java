// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.intakeSubsystem;

public class runFlyWheelsTime extends Command {
  //^ Auto helper that runs the Flywheel system for a given time at a given speed
  private intakeSubsystem m_IntakeSystem;
  private Timer timer;
  private double m_time;
  private double m_speed;
  public runFlyWheelsTime(double speed, double time) {
    m_IntakeSystem = Robot.m_IntakeSubsystem;
    addRequirements(m_IntakeSystem);
    m_time = time;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Start a timer
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Set the flywheel speeds
    m_IntakeSystem.setFlyWheelSpeeds(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.setFlyWheelSpeeds(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //End if the timer has passed the given time
    return timer.hasElapsed(m_time);
  }
}

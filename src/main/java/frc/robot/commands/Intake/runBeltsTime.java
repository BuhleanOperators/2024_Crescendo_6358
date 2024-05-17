// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.beltSubsystem;

public class runBeltsTime extends Command {
  //^ Auto helper that runs the Belt system for a given time at a given speed
  private beltSubsystem m_BeltSubsystem;
  private double m_speed;
  private double m_time;
  private Timer timer;
  public runBeltsTime(double speed, double time) {
    m_BeltSubsystem = Robot.m_BeltSubsystem;
    m_time = time;
    m_speed = speed;
    addRequirements(m_BeltSubsystem);
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
    //Set the speed of the belts
    m_BeltSubsystem.setBeltSpeeds(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_BeltSubsystem.setBeltSpeeds(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //End if the timer has passed the given time
    return timer.hasElapsed(m_time);
  }
}

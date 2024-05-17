// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.shooterSubsystem;

public class shooterRun extends Command {
  //^ Set the shooter at given speed
  private double m_speed;
  private shooterSubsystem m_shooterSystem;
  public shooterRun(double speed) {
    m_speed = speed;
    m_shooterSystem = Robot.m_ShooterSubsytem;
    addRequirements(m_shooterSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Set motors to given speed
    m_shooterSystem.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

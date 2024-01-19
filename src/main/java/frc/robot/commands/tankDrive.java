// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveTrain;

public class tankDrive extends Command {
  /** Creates a new tankDrive. */
  private static driveTrain m_DriveTrain;
  private double m_xSpeed;
  private double m_rot;
  public tankDrive(double xSpeed, double rot, driveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_xSpeed = xSpeed;
    m_rot = rot;
    m_DriveTrain = subsystem;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveTrain.drive(m_xSpeed, m_rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

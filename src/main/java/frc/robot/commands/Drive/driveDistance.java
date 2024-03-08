// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.driveTrain;

public class driveDistance extends Command {
  /** Creates a new DriveDistance. */
  private driveTrain m_DriveTrain;
  private double m_target;
  public driveDistance(double target) {
    m_target = target;
    m_DriveTrain = Robot.m_DriveTrain;
    addRequirements(m_DriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveTrain.setAutoSpeeds(m_target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_DriveTrain.getAverageDistance()) >= Math.abs(m_target);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveTrain;

public class turnAngle extends Command {
  /** Creates a new trunAngle. */
  private double m_angle;
  private driveTrain m_driveTrain;
  private ADIS16448_IMU m_gyro = m_driveTrain.getGyro();
  public turnAngle(double angle, driveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_angle = angle;
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double goal = m_gyro.getGyroAngleX() + m_angle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

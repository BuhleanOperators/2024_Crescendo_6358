// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveTrain;

public class newDrive extends Command {
  /** Creates a new newDrive. */
  private DoubleSupplier m_xSpeed;
  private DoubleSupplier m_rot;
  private driveTrain m_Drive;
  public newDrive(DoubleSupplier xSpeed, DoubleSupplier rot, driveTrain subsytem) {
    m_xSpeed = xSpeed;
    m_rot = rot;
    m_Drive =  subsytem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drive.newDrive(m_xSpeed.getAsDouble(), m_rot.getAsDouble());
    SmartDashboard.putNumber("Right Encoder Value", m_Drive.getRightEncoder().getPosition());
    SmartDashboard.putNumber("Left Encoder Value", m_Drive.getLeftEncoder().getPosition());
    SmartDashboard.putNumber("Right Conversion Factor", m_Drive.getRightEncoder().getPositionConversionFactor());
    SmartDashboard.putNumber("Left Conversion Factor", m_Drive.getLeftEncoder().getPositionConversionFactor());
    SmartDashboard.putNumber("Right Distance", m_Drive.getRightEncoderDistance());
    SmartDashboard.putNumber("Left Distance", m_Drive.getLeftEncoderDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

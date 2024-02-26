// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveTrain;

public class arcadeDrive extends Command {
  /** Creates a new arcadeDrive. */
  private DoubleSupplier m_xSpeed;
  private DoubleSupplier m_rot;
  private driveTrain m_DriveTrain;
  public arcadeDrive(DoubleSupplier xSpeed, DoubleSupplier rot, driveTrain subsystem) {
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
    m_DriveTrain.arcadeDrive(m_xSpeed.getAsDouble(), m_rot.getAsDouble());
    SmartDashboard.putNumber("Right Encoder Value", m_DriveTrain.getRightEncoder().getPosition());
    SmartDashboard.putNumber("Left Encoder Value", m_DriveTrain.getLeftEncoder().getPosition());
    SmartDashboard.putNumber("Right Conversion Factor", m_DriveTrain.getRightEncoder().getPositionConversionFactor());
    SmartDashboard.putNumber("Left Conversion Factor", m_DriveTrain.getLeftEncoder().getPositionConversionFactor());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

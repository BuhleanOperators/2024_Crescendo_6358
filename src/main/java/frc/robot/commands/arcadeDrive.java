// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveTrain;
import frc.robot.Constants.DriveConstants;;

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
    m_DriveTrain.arcadeDrive(m_xSpeed.getAsDouble() * DriveConstants.maxSpeed, m_rot.getAsDouble() * DriveConstants.maxAngularSpeed);
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

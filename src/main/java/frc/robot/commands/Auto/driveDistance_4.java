// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveTrain;

public class driveDistance_4 extends Command {
  /** Creates a new driveDistance_3. */
  private driveTrain m_DriveTrain;
  private double m_speed;
  private double m_distance;
  public driveDistance_4(double speed, double distance, driveTrain subsystem) {
    m_DriveTrain = subsystem;
    m_speed = speed;
    m_distance = distance;
    addRequirements(m_DriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveTrain.resetEncoders();
    m_DriveTrain.arcadeDrive(m_speed, 0);
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
    return m_DriveTrain.getRightEncoderDistance() <= m_distance;
  }
}

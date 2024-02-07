// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveTrain;

public class driveDistance_kinematics extends Command {
  /** Creates a new driveDistance_kinematics. */
  private driveTrain m_driveTrain;
  private double distance;
  private double targetDistance;
  public driveDistance_kinematics(double distance) {
    this.distance = distance;
    m_driveTrain = new driveTrain();
    addRequirements(m_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetDistance = m_driveTrain.getAverageDistance() + distance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.autoDrive(targetDistance, 0);
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

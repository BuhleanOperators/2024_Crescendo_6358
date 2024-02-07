// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveTrain;

public class driveToDistance extends Command {
  /** Creates a new driveToDistance. */
  private driveTrain m_DriveTrain;
  private double initialDistance;
  private double distance;
  private double percentPower;
  public driveToDistance(double distance, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveTrain = new driveTrain();
    this.distance = distance;
    percentPower = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //initialDistance = m_DriveTrain.getRightDistance();
    m_DriveTrain.arcadeDrive(percentPower, 0);
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
    return false;
    //return m_DriveTrain.getRightDistance() >= initialDistance + distance;
  }
}

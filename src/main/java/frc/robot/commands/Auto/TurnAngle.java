// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveTrain;

public class TurnAngle extends Command {
  /** Creates a new TurnAngle. */
  private driveTrain m_DriveTrain;
  private double m_angle;
  private double m_speeds;
  public TurnAngle(double angle, double speeds, driveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_angle = angle;
    m_speeds = speeds;
    m_DriveTrain = subsystem;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO make sure that everything goes in the right direction; positive direction positive speed
    m_DriveTrain.resetGyro();
    m_DriveTrain.setAutoSpeeds(m_speeds);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Gyro Angle", m_DriveTrain.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_DriveTrain.getAngle()) >= Math.abs(m_angle);
  }
}

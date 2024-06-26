// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.driveTrain;

public class turnAngle extends Command {
  //^ Auto helper to turn to a target angle
  private driveTrain m_DriveTrain;
  private double m_angle;
  private double m_speed;
  public turnAngle(double angle, double speed) {
    m_angle = angle;
    m_speed = speed;
    m_DriveTrain = Robot.m_DriveTrain;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Reset the gyro then set the target angle
    m_DriveTrain.resetGyro();
    m_DriveTrain.setAutoTurnSpeeds(m_speed);
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
    //End if the absolute value of the current angle is greater than or equal to the target angle
    return Math.abs(m_DriveTrain.getAngle()) >= Math.abs(m_angle);
  }
}

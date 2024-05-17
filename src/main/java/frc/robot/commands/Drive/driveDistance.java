// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.driveTrain;

public class driveDistance extends Command {
  //^ Auto helper to drive forward a set distance
  private driveTrain m_DriveTrain;
  private double m_target;
  public driveDistance(double target) {
    m_target = target;
    m_DriveTrain = Robot.m_DriveTrain;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Reset the encodes, wait 3/4 a second to allow encoders to reset, set target distance
    Robot.m_DriveTrain.resetEncoders();
    new WaitCommand(0.75);
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
    //End if the absolute value of the driven distance is greater than or equal to the value of the target
    return Math.abs(m_DriveTrain.getAverageDistance()) >= Math.abs(m_target);
  }
}

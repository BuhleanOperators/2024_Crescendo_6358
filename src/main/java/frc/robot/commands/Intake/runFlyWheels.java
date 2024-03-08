// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.intakeSubsystem;

public class runFlyWheels extends Command {
  /** Creates a new runIntake. */
  private intakeSubsystem m_IntakeSystem;
  private double m_speed;
  
  public runFlyWheels(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speed = speed;
    m_IntakeSystem = Robot.m_IntakeSubsystem;
    addRequirements(m_IntakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSystem.setFlyWheelSpeeds(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.setFlyWheelSpeeds(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

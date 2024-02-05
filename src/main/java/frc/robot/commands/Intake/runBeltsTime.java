// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSystem;

public class runBeltsTime extends Command {
  /** Creates a new runBeltsTime. */
  private intakeSystem m_IntakeSystem;
  private double m_speed;
  private double m_time;
  private Timer timer;
  public runBeltsTime(double speed, double time) {
    // Use addRequirements() here to declare subsystem dependencies.m_speed = speed;
    m_IntakeSystem = new intakeSystem();
    m_time = time;
    m_speed = speed;
    addRequirements(m_IntakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSystem.setBeltSpeeds(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.setBeltSpeeds(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(m_time);
  }
}
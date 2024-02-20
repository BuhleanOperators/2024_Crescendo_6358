// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSystem;

public class fullIntake extends Command {
  /** Creates a new fullIntake. */
  private intakeSystem m_IntakeSystem;
  private double m_flySpeed;
  private double m_beltSpeed;
  public fullIntake(double flySpeed, double beltSpeed, intakeSystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_flySpeed = flySpeed;
    m_beltSpeed = beltSpeed;
    m_IntakeSystem = subsystem;
    addRequirements(m_IntakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSystem.setBeltSpeeds(m_beltSpeed);
    m_IntakeSystem.setFlyWheelSpeeds(m_flySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.setBeltSpeeds(0);
    m_IntakeSystem.setFlyWheelSpeeds(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

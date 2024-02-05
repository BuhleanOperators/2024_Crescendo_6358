// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.phaseTwoSystem;

public class phase2 extends Command {
  /** Creates a new phase2. */
  private phaseTwoSystem m_PhaseTwoSystem;
  private double m_speed;
  public phase2(double speed, phaseTwoSystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speed = speed;
    m_PhaseTwoSystem = subsystem;
    addRequirements(m_PhaseTwoSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_PhaseTwoSystem.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_PhaseTwoSystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

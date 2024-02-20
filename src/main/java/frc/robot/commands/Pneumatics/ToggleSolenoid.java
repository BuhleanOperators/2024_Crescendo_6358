// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pneumatics;

public class ToggleSolenoid extends Command {
  /** Creates a new ToggleSolenoid. */
  private DoubleSolenoid m_piston;
  private pneumatics m_pneumatics;
  public ToggleSolenoid(DoubleSolenoid piston, pneumatics subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_piston = piston;
    m_pneumatics = subsystem;
    addRequirements(m_pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pneumatics.toggleSolenoid(m_piston);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
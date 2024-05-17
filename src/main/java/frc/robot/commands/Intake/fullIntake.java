// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.beltSubsystem;
import frc.robot.subsystems.intakeSubsystem;

public class fullIntake extends Command {
  //^ Run the Flywheel and Belt systems simultaniously at separate given speeds
  private intakeSubsystem m_IntakeSystem;
  private beltSubsystem m_BeltSubsystem;
  private double m_flySpeed;
  private double m_beltSpeed;
  public fullIntake(double flySpeed, double beltSpeed) {
    m_flySpeed = flySpeed;
    m_beltSpeed = beltSpeed;
    m_IntakeSystem = Robot.m_IntakeSubsystem;
    m_BeltSubsystem = Robot.m_BeltSubsystem;
    addRequirements(m_IntakeSystem, m_BeltSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Set the speeds of the belts and flywheels
    m_BeltSubsystem.setBeltSpeeds(m_beltSpeed);
    m_IntakeSystem.setFlyWheelSpeeds(m_flySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_BeltSubsystem.setBeltSpeeds(0);
    m_IntakeSystem.setFlyWheelSpeeds(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;

public class Orange extends Command {
  /** Creates a new Orange. */
  private final LED led;
  public Orange(LED subsystem) {
    addRequirements(subsystem);
    led = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.setOrange();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

}

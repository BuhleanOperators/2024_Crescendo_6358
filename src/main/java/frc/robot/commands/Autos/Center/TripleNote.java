// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Center;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.Other.shootSpeaker;
import frc.robot.commands.Drive.driveDistance;
import frc.robot.commands.Drive.reverseDistance;
import frc.robot.commands.Intake.fullIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TripleNote extends SequentialCommandGroup {
  /** Creates a new TripleNote. */
  public TripleNote() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DoubleSpeakerCenter()
      // new 
    );
  }
}

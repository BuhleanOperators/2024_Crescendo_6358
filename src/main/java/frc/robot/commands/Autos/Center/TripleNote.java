// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Center;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TripleNote extends SequentialCommandGroup {
  //^ Score preloaded NOTE, leave, gether NOTE directly behind robot, score NOTE, turn to gather NOTE to side of robot, score NOTE
  //! INCOMPLETE
  public TripleNote() {
    addCommands(
      new DoubleSpeakerCenter()
      // new 
    );
  }
}

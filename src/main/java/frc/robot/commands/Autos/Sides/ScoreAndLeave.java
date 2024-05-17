// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Sides;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.Other.shootSpeaker;
import frc.robot.commands.Drive.driveDistance;
import frc.robot.commands.Drive.turnAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAndLeave extends SequentialCommandGroup {
  //^ Score proloaded NOTE, leave
  public ScoreAndLeave(int multiplier) {
    addCommands(
      new shootSpeaker(),
      new driveDistance(1),
      new turnAngle(62 * multiplier, 0.85),
      new driveDistance(5)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Sides;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.Other.shootSpeaker;
import frc.robot.commands.Drive.driveDistance;
import frc.robot.commands.Drive.reverseDistance;
import frc.robot.commands.Drive.turnAngle;
import frc.robot.commands.Intake.fullIntake;
import frc.robot.commands.Intake.runBelts;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleSpeaker extends SequentialCommandGroup {
  //^ Score preloaded NOTE, leave, gather NOTE behind robot, return, score NOTE
  //! INCONSISTANT
  public DoubleSpeaker(int multiplier) {
    addCommands(
      new shootSpeaker(),
      new driveDistance(1.25),
      new turnAngle(62 * multiplier, 0.85 * multiplier),
      new driveDistance(4),
      new ParallelCommandGroup(
        new driveDistance(0.5),
        new fullIntake(1, 1).withTimeout(0.5)
      ),
      new ParallelCommandGroup(
        new runBelts(-0.5).withTimeout(1),
        new reverseDistance(4.25)
      ),
      new turnAngle(62 * -multiplier, 0.85 * -multiplier),
      // new reverseDistance(1.3), //? Does it need this or can it make it anyway?
      new shootSpeaker()
    );
  }
}

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
import frc.robot.commands.Intake.runBelts;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleSpeakerCenter extends SequentialCommandGroup {
  /** Creates a new DoubleSpeakerCenter. */
  public DoubleSpeakerCenter() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new shootSpeaker(),
      new driveDistance(3.5),
      new ParallelCommandGroup(
        new driveDistance(0.75),
        new fullIntake(1, 1).withTimeout(0.75)),
      new ParallelCommandGroup(
        new reverseDistance(4),
        new runBelts(-1).withTimeout(1)
      ),
      new shootSpeaker()
    );
  }
}

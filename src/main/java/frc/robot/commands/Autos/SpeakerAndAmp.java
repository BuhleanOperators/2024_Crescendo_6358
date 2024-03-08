// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Drive.driveDistance;
import frc.robot.commands.Drive.turnAngle;
import frc.robot.commands.Intake.fullIntake;
import frc.robot.commands.Intake.runBelts;
import frc.robot.commands.Shooter.shooterRun;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerAndAmp extends SequentialCommandGroup {
  /** Creates a new SpeakerAndAmp. */
  public SpeakerAndAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new shootOnly().withTimeout(4),
      new driveDistance(1.5),
      new turnAngle(62, 0.75),
      new driveDistance(2),
      new fullIntake(1, 1).withTimeout(1),
      new runBelts(-0.5).withTimeout(1),
      new driveDistance(-1),
      new turnAngle(90, 0.75),
      new driveDistance(-2),
      new shooterRun(ShooterConstants.ampSpeed).withTimeout(3),
      new ParallelCommandGroup(
        new runBelts(1),
        new shooterRun(ShooterConstants.ampSpeed)
      ).withTimeout(5)
    );
  }
}

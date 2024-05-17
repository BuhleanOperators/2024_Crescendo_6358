// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Other;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Intake.runBelts;
import frc.robot.commands.Shooter.shooterRun;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootSpeaker extends SequentialCommandGroup {
  /** Creates a new shootOnly. */
  public shootSpeaker() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new shooterRun(ShooterConstants.speakerSpeed).until(() -> Robot.m_ShooterSubsytem.getShooterRPM() >= ShooterConstants.speakerRPM),
      new ParallelCommandGroup(
        new runBelts(1),
        new shooterRun(.85)
      ).withTimeout(1)
    );
  }
}

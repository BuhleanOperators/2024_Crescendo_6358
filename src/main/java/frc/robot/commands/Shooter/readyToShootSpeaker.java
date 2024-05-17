// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Intake.runBelts;
import frc.robot.commands.Intake.runFlyWheels;
import frc.robot.subsystems.shooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class readyToShootSpeaker extends SequentialCommandGroup {
  //^ Auto help that sets up to score in the SPEAKER
  private shooterSubsystem m_ShooterSystem;
  public readyToShootSpeaker() {
    addCommands(
      new runBelts(1).withTimeout(2),
      new runBelts(-0.5).withTimeout(0.5),
      new runFlyWheels(ShooterConstants.speakerSpeed).until(() -> (m_ShooterSystem.getShooterRPM() >= ShooterConstants.speakerRPM)),
      new ParallelCommandGroup(
        new runFlyWheels(ShooterConstants.speakerSpeed),
        new runBelts(1)
      )
    );
  }
}

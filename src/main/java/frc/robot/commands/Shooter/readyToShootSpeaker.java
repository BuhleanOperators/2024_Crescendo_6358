// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Intake.runBelts;
import frc.robot.commands.Intake.runFlyWheels;
import frc.robot.subsystems.beltSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class readyToShootSpeaker extends SequentialCommandGroup {
  /** Creates a new readyToShootSpeaker. */
  private intakeSubsystem m_IntakeSystem;
  private shooterSubsystem m_ShooterSystem;
  private beltSubsystem m_BeltSubsystem;
  public readyToShootSpeaker() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new runBelts(1, m_BeltSubsystem).withTimeout(2),
      new runBelts(-0.5, m_BeltSubsystem).withTimeout(0.5),
      new runFlyWheels(ShooterConstants.speakerSpeed, m_IntakeSystem).until(() -> (m_ShooterSystem.getShooterRPM() >= ShooterConstants.fullRPM)),
      new ParallelCommandGroup(
        new runFlyWheels(ShooterConstants.speakerSpeed, m_IntakeSystem),
        new runBelts(1, m_BeltSubsystem)
      )
    );
  }
}

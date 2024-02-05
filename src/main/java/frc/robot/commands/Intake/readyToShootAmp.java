// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.intakeSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class readyToShootAmp extends SequentialCommandGroup {
  /** Creates a new readyToShoot. */
  private intakeSystem m_IntakeSystem;
  public readyToShootAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new runBelts(1, m_IntakeSystem),
      new WaitUntilCommand(0),
      new runBelts(-0.5, m_IntakeSystem),
      new WaitCommand(0),
      new runFlyWheels(0, m_IntakeSystem),
      new WaitCommand(0),
      new ParallelCommandGroup(
        new runFlyWheels(ShooterConstants.ampSpeed, m_IntakeSystem),
        new runBelts(0, m_IntakeSystem)
      )
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoContants;
import frc.robot.subsystems.driveTrain;

public class driveDistance_7587 extends PIDCommand {
  /** Creates a new driveDistance_7587. */
  private final driveTrain driveTrain;
  private final double target;
  public driveDistance_7587(driveTrain driveTrain, double distance, double current) {
    // Use addRequirements() here to declare subsystem dependencies.
    super( new PIDController(AutoContants.DIST_P, AutoContants.DIST_I, AutoContants.DIST_D),
    () -> driveTrain.getAverageDistance(),
    current + distance,
    (output) -> driveTrain.arcadeDrive(0, output),
    driveTrain);

    getController().setTolerance(AutoContants.DIST_distanceTolerance, AutoContants.DIST_velocityTolerance);

    this.driveTrain = driveTrain;
    this.target = current + distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrain.getAverageDistance() >= target - 2 && driveTrain.getAverageDistance() <= target + 2;
  }
}

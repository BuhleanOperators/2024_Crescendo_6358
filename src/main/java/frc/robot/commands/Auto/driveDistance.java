// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoContants;
import frc.robot.subsystems.driveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class driveDistance extends PIDCommand {
  /** Creates a new driveDistance. */
  public driveDistance(double distance, driveTrain drive) {
    super(
        // The controller that the command will use
        new PIDController(AutoContants.DIST_P, AutoContants.DIST_I, AutoContants.DIST_D),
        // This should return the measurement
        () -> drive.getRightEncoder().getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> distance,
        // This uses the output
        output -> {
          drive.arcadeDrive(output, 0);
        }, drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(AutoContants.DIST_distanceTolerance, AutoContants.DIST_velocityTolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

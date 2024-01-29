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
public class turnAngle extends PIDCommand {
  /** Creates a new turnAngle. */
  public turnAngle(double targetAngle, driveTrain drive) {
    super(
        // The controller that the command will use
        new PIDController(AutoContants.TURN_P, AutoContants.TURN_I, AutoContants.TURN_D),
        // This should return the measurement
        () -> drive.getHeading(),
        // This should return the setpoint (can also be a constant)
        () -> targetAngle,
        // This uses the output
        output -> {
          drive.arcadeDrive(0, output);
        }, drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(AutoContants.TURN_turnToleranceDeg, AutoContants.TURN_turnRateToleranceDegPerSec);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

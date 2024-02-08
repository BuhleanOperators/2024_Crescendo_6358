// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.driveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class driveDistance extends PIDCommand {
  /** Creates a new driveDistance. */
  private static driveTrain m_drive;
  private final double target;
  public driveDistance(double distance, double current) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> m_drive.getAverageDistance(),
        // This should return the setpoint (can also be a constant)
        () -> current + distance,
        // This uses the output
        output -> {
          // Use the output here
          m_drive.newDrive(output, 0);
        }, m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_drive = new driveTrain();
    this.target = current + distance;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getAverageDistance() >= target - 2 && m_drive.getAverageDistance() <= target + 2;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoContants;
import frc.robot.subsystems.driveTrain;

public class turnToAngle extends Command {
  /** Creates a new turnToAngle3. */
  private driveTrain m_DriveTrain;
  private double degreesToTurn;
  private double error;
  private double targetAngle;
  public turnToAngle(double degreesToTurn) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveTrain = new driveTrain();
    addRequirements(m_DriveTrain);
    this.degreesToTurn = degreesToTurn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.targetAngle = degreesToTurn + m_DriveTrain.getHeading();
    //System.out.println("Current Angle:" + m_DriveTrain.getHeading());
    //System.out.println("Target Angle:" + targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = targetAngle - m_DriveTrain.getHeading();
    
    double value = error * AutoContants.TURN_P;
    if (Math.abs(value) > 0.75){
      value = Math.copySign(0.75, value);
    }
    if (Math.abs(value) < 0.15){
      value = Math.copySign(0.15, value);
    }
    //System.out.println("Error:" + error);
    //System.out.println("value:" + value);
    m_DriveTrain.arcadeDrive(0, value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < AutoContants.TURN_turnTolerance;
  }
}

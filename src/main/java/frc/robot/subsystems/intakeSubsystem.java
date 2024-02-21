// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class intakeSubsystem extends SubsystemBase {
  /** Creates a new intakeSystem. */
  private VictorSP intake;

  public intakeSubsystem() {
    intake = new VictorSP(IntakeConstants.intakeID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFlyWheelSpeeds(double speed){
    intake.set(speed);
  }
}

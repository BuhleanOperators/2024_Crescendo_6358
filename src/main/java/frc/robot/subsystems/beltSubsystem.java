// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class beltSubsystem extends SubsystemBase {
  //^ Initialization and methods for Belt system
  private VictorSP belts;
  public beltSubsystem() {
    //& VictorSP
    belts = new VictorSP(IntakeConstants.beltID); //Create the contoller for the belts
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //^ Void Methods
  public void setBeltSpeeds(double speed){
    //Set belts to a gaven speed
    belts.set(speed);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class intakeSystem extends SubsystemBase {
  /** Creates a new intakeSystem. */
  private Spark intake;

  public intakeSystem() {
    intake = new Spark(3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeeds(double speed){
    intake.set(speed);
  }
}

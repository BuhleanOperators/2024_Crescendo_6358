// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
  /** Creates a new shooter. */
  public Spark spark1;
  public Spark spark2;
  public shooter() {
    spark1 = new Spark(2);
    spark2 = new Spark(3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){
    spark1.set(speed);
    spark2.set(speed);
  }
}

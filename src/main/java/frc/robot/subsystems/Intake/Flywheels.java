// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheels extends SubsystemBase {
  /** Creates a new Flywheels. */
  private FlywheelsIO io;

  public Flywheels(FlywheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //**Runs the motor at a specified speed */
  public void runSpeed(double speed){
    io.setSpeed(speed);
  }

  //**Stops the motor */
  public void stop(){
    io.stop();
  }
}

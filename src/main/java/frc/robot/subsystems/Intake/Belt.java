// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Belt extends SubsystemBase {
  /** Creates a new Belt. */
  private final BeltIO io;

  public Belt(BeltIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //**Run the belt motor at specified speed */
  public void runSpeed(double speed){
    io.setSpeed(speed);
  }

  //**Stop the motor */
  public void stopMotor(){
    io.stop();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pneumatics;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  private PneumaticsIO io;
  private PneumaticsIOInputsAutoLogged inputs = new PneumaticsIOInputsAutoLogged();

  public Pneumatics(PneumaticsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    Logger.processInputs("Pneumatics", inputs);

    io.enableCompressor(100, 120);
  }

  //**Set the position of the climb solenoids to forward */
  public void climbUp (){
    io.setPosition(Value.kForward);
  }

  //**Set the position of the climb solenoids to reverse */
  public void climbDown(){
    io.setPosition(Value.kReverse);
  }

  //**Toggle the position of climb solenoids */
  public void toggleClimb(){
    io.togglePosition();
  }

  public boolean isClimbUp(){
    return io.climbValue() == DoubleSolenoid.Value.kForward;
  }
}

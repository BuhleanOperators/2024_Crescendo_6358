// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class pneumaticSubsystem extends SubsystemBase {
  /** Creates a new pneumatics. */
  private DoubleSolenoid firstSolenoid;
  private DoubleSolenoid secondSolenoid;
  public pneumaticSubsystem() {
    firstSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.PISTON_FIRST_FORWARD, PneumaticsConstants.PISTON_FIRST_BACK);
    secondSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.PISTON_SECOND_FORWARD, PneumaticsConstants.PISTON_SECOND_BACK);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public DoubleSolenoid getFirstSolenoid(){
    return firstSolenoid;
  }
  public DoubleSolenoid getSecondSolenoid(){
    return secondSolenoid;
  }

  public void fireSolenoid(DoubleSolenoid piston){
    piston.set(DoubleSolenoid.Value.kForward);
  }
  public void retractSolenoid(DoubleSolenoid piston){
    piston.set(DoubleSolenoid.Value.kReverse);
  }

  public void toggleSolenoid(DoubleSolenoid piston){
    piston.toggle();
  }
}

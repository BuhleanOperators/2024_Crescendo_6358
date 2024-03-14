// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class pneumaticSubsystem extends SubsystemBase {
  /** Creates a new pneumatics. */
  private PneumaticHub m_pH = new PneumaticHub(10);
  private DoubleSolenoid rightSolenoid;
  private DoubleSolenoid leftSolenoid;
  public pneumaticSubsystem() {
    rightSolenoid = m_pH.makeDoubleSolenoid(PneumaticsConstants.PISTON_RIGHT_FORWARD, PneumaticsConstants.PISTON_RIGHT_BACK);
    leftSolenoid = m_pH.makeDoubleSolenoid(PneumaticsConstants.PISTON_LEFT_FORWARD, PneumaticsConstants.PISTON_LEFT_BACK);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public DoubleSolenoid getRightSolenoid(){
    return rightSolenoid;
  }
  public DoubleSolenoid getLeftSolenoid(){
    return leftSolenoid;
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
  public void compressorOn(){
    m_pH.enableCompressorAnalog(100, 120);
  }
}

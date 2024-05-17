// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class pneumaticSubsystem extends SubsystemBase {
  //^ Initialization and methods for the pneumatics system
  private PneumaticHub m_pH = new PneumaticHub(10); //Create the pneumatic hub
  private DoubleSolenoid rightSolenoid;
  private DoubleSolenoid leftSolenoid;
  public pneumaticSubsystem() {
    rightSolenoid = m_pH.makeDoubleSolenoid(PneumaticsConstants.PISTON_RIGHT_FORWARD, PneumaticsConstants.PISTON_RIGHT_BACK); //Create a solinoid
    leftSolenoid = m_pH.makeDoubleSolenoid(PneumaticsConstants.PISTON_LEFT_FORWARD, PneumaticsConstants.PISTON_LEFT_BACK); //Create a solinoid
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  //^ Accsessor Methods
  public DoubleSolenoid getRightSolenoid(){
    return rightSolenoid;
  }
  public DoubleSolenoid getLeftSolenoid(){
    return leftSolenoid;
  }

  //^ Void Methods
  public void fireSolenoid(DoubleSolenoid piston){
    //Set the position of a soilinoid to forward
    piston.set(DoubleSolenoid.Value.kForward);
  }
  public void retractSolenoid(DoubleSolenoid piston){
    //Set the position of a soilinoid to reverse
    piston.set(DoubleSolenoid.Value.kReverse);
  }

  public void toggleSolenoid(DoubleSolenoid piston){
    //Toggle a soilinoid between forward and reverse 
    piston.toggle();
  }
  public void compressorOn(){
    //Turn on the compressor
    m_pH.enableCompressorAnalog(100, 120);
  }
}

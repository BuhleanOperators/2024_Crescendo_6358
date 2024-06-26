// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class ledSubsystem extends SubsystemBase {
  //^ Initialization and methods for the LEDs
  //! DOESN'T WORK
  private AddressableLED m_Led;
  private AddressableLEDBuffer m_Buffer;
  public ledSubsystem() {
    m_Led = new AddressableLED(LedConstants.port);
    m_Buffer = new AddressableLEDBuffer(LedConstants.length);

    m_Led.setLength(m_Buffer.getLength());
    m_Led.start();
    m_Led.setData(m_Buffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  //^ Accsessor Methods
  public AddressableLED getLed(){
    //Get the LEDs
    return m_Led;
  }

  //^ Solid Color Set 
  public void off(){
    //Turn off the LED strip
    for (var i = 0; i < m_Buffer.getLength(); i++){
      m_Buffer.setRGB(i, 0, 0, 0);
    }
    m_Led.setData(m_Buffer);
  }
  public void setRed(){
    //Set the LED strip to red
    for (var i = 0; i < m_Buffer.getLength(); i++){
      m_Buffer.setRGB(i, 255, 0, 0);
    }
    m_Led.setData(m_Buffer);
  }
  public void setBlue(){
    //Set the LED strip to blue
    for (var i = 0; i < m_Buffer.getLength(); i++){
      m_Buffer.setRGB(i, 0, 0, 255);
    }
    m_Led.setData(m_Buffer);
  }
  public void setGreen(){
    //Set the LED strip to green
    for (var i = 0; i < m_Buffer.getLength(); i++){
      m_Buffer.setRGB(i, 0, 255, 0);
    }
    m_Led.setData(m_Buffer);
  }
  public void setOrange(){
    //Set the LED strip to orange
    for (var i = 0; i < m_Buffer.getLength(); i++){
      m_Buffer.setRGB(i, 253, 120, 0);
    }
    m_Led.setData(m_Buffer);
  }

  //^ Non-Solid Color Set



}

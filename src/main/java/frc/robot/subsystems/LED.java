// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private int FirstPixelHue = 0;
  public LED(int m_port, int m_length) {
    led = new AddressableLED(m_port);
    buffer = new AddressableLEDBuffer(m_length);
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }

  public void setBlue(){
    for (var i = 0; i < buffer.getLength(); i++){
      buffer.setRGB(i, 0, 0, 255);
    }
    led.setData(buffer);
  }
  public void setRed(){
    for (var i = 0; i < buffer.getLength(); i++){
      buffer.setRGB(i, 255, 0, 0);
    }
    led.setData(buffer);
  }

  public void setOrange(){
    for (var i = 0; i < buffer.getLength(); i++){
      buffer.setRGB(i, 253, 120, 0);
    }
    led.setData(buffer);
  }
  public void off(){
    for (var i = 0; i < buffer.getLength(); i++){
      buffer.setRGB(i, 0, 0, 0);
    }
    led.setData(buffer);
  }
}

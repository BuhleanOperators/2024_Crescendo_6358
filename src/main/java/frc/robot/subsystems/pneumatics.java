// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class pneumatics extends SubsystemBase {
  /** Creates a new pneumatics. */
  private Compressor mCompressor;
  public pneumatics() {
    mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void enableCompressor(){
    mCompressor.enableAnalog(0, 0);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pneumatics;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/** Add your docs here. */
public interface PneumaticsIO {
    @AutoLog
    public static class PneumaticsIOInputs{
        public DoubleSolenoid.Value leftSolinoidValue = DoubleSolenoid.Value.kOff;
        public DoubleSolenoid.Value rightSolinoidValue = DoubleSolenoid.Value.kOff;
        public boolean compressorEnabled = false;
        public double pressure = 0.0;
    }

    //**Updates the set of updatable inputs */
    public default void updateInputs(PneumaticsIOInputs inputs){}

    //**Set the position of a solinoid to specified direction */
    public default void setPosition(DoubleSolenoid.Value direction){}

    //**Enable compressor with specified minimum and maximum pressures */
    public default void enableCompressor(double minPressurePSI, double maxPressurePSI){}

    //**Toggle the position of a solinoid */
    public default void togglePosition(){}

    public default DoubleSolenoid.Value climbValue(){return null;}
}

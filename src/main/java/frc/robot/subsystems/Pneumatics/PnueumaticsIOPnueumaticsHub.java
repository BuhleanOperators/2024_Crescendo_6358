// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.Constants.PneumaticsConstants;

/** Add your docs here. */
public class PnueumaticsIOPnueumaticsHub implements PneumaticsIO{
    private PneumaticHub pneumaticHub = new PneumaticHub();
    private DoubleSolenoid leftSolenoid;
    private DoubleSolenoid rightSolenoid;
    private Compressor compressor;

    public PnueumaticsIOPnueumaticsHub(){
        leftSolenoid = pneumaticHub.makeDoubleSolenoid(PneumaticsConstants.PISTON_LEFT_FORWARD, PneumaticsConstants.PISTON_LEFT_BACK);
        rightSolenoid = pneumaticHub.makeDoubleSolenoid(PneumaticsConstants.PISTON_RIGHT_FORWARD, PneumaticsConstants.PISTON_RIGHT_BACK);
        compressor = pneumaticHub.makeCompressor();
    }

    @Override
    public void updateInputs(PneumaticsIOInputs inputs){
        inputs.leftSolinoidValue = leftSolenoid.get();
        inputs.rightSolinoidValue = rightSolenoid.get();
        inputs.compressorEnabled = pneumaticHub.getCompressor();
        inputs.pressure = pneumaticHub.getPressure(0);
    }

    @Override
    public void setPosition(DoubleSolenoid.Value position){
        leftSolenoid.set(position);
        rightSolenoid.set(position);
    }

    @Override
    public void enableCompressor(double minPressurePSI, double maxPressurePSI){
        compressor.enableAnalog(minPressurePSI, maxPressurePSI);
    }

    @Override
    public void togglePosition(){
        leftSolenoid.toggle();
        rightSolenoid.toggle();
    }

    @Override
    public DoubleSolenoid.Value climbValue(){
        return leftSolenoid.get();
    }

}

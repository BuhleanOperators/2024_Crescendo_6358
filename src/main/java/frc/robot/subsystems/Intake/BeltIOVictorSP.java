// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class BeltIOVictorSP implements BeltIO {
    private VictorSP motor = new VictorSP(IntakeConstants.beltID);

    public BeltIOVictorSP(){
    }

    @Override
    public void updateInputs(BeltIoInputs inputs){
        motor.setInverted(false);
    }

    @Override
    public void setSpeed(double speed){
        motor.set(speed);
    }

    @Override
    public void stop(){
        motor.stopMotor();
    }
}

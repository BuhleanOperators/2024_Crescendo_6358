// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos.Center.DoubleSpeakerCenter;
import frc.robot.commands.Autos.Other.Test;
import frc.robot.commands.Autos.Other.noAuto;
import frc.robot.commands.Autos.Other.shootSpeaker;
import frc.robot.commands.Autos.Sides.DoubleSpeaker;
import frc.robot.commands.Autos.Sides.ScoreAndLeave;
import frc.robot.commands.Autos.Sides.SpeakerAndAmp;

/** Add your docs here. */
public class smartDashboard {
    //^ Class made to put values to SmartDashboard
    private static SendableChooser<String> allianceColor = new SendableChooser<>();
    private static SendableChooser<Command> autoChooser = new SendableChooser<>();
    private int multiplier;

    //Add options to the dashboard of what alliance we are on
    public void AllianceColor(){
        allianceColor.setDefaultOption("Non-Event", "orange");
            allianceColor.addOption("Red Alliance", "red");
            allianceColor.addOption("Blue Alliance", "blue");
        
        SmartDashboard.putData(allianceColor);
    }
    //Add options to the dashboard of what auto to run
    public void AutoChooser(){
        autoChooser.setDefaultOption("No Auto", new noAuto());
            autoChooser.addOption("Speaker Only", new shootSpeaker());
            autoChooser.addOption("Shoot and Leave", new ScoreAndLeave(multiplier));
            autoChooser.addOption("Speaker and Amp", new SpeakerAndAmp(multiplier));
            autoChooser.addOption("Shoot and Turn", new Test(multiplier));
            autoChooser.addOption("Double Speaker", new DoubleSpeaker(multiplier));
            autoChooser.addOption("Center Double Speaker", new DoubleSpeakerCenter());

        SmartDashboard.putData(autoChooser);
    }
    //Gather robot data and put it to the dashboard
    public void gatherData(){
        SmartDashboard.putNumber("Distance", Robot.m_DriveTrain.getAverageDistance());
        SmartDashboard.putNumber("Angle", Robot.m_DriveTrain.getAngle());
        SmartDashboard.putNumber("Shooter RPM", Robot.m_ShooterSubsytem.getShooterRPM());
        SmartDashboard.putNumber("Multiplier", multiplier);
    }
    public void getLedColor(){
    }

    //Set the multiplier
    public void multiplier(){
        if (allianceColor.getSelected().equals("red")){
            multiplier = 1;
        }else if(allianceColor.getSelected().equals("blue")){
            multiplier = -1;
        }else {
            multiplier = 0;
        }
    }
    //If the shooter system is up to speed to score in the SPEAKER show that on the dashboard
    public void upToSpeed(){
        if (Robot.m_ShooterSubsytem.getShooterRPM() >= ShooterConstants.speakerRPM){
            SmartDashboard.putBoolean("", true);
        }else {
            SmartDashboard.putBoolean("", false);
        }
    }


    //^ Accsessor Methods
    public SendableChooser getAllianceColor(){
        return allianceColor;
    }
    public SendableChooser getAutoChooser(){
        return autoChooser;
    }
    public Command getAutoCommand(){
        return autoChooser.getSelected();
    }
}

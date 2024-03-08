// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autos.noAuto;
import frc.robot.commands.Autos.shootOnly;
import frc.robot.commands.Drive.driveDistance;
import frc.robot.subsystems.driveTrain;

/** Add your docs here. */
public class smartDashboard {
    private static SendableChooser<String> allianceColor = new SendableChooser<>();
    private static SendableChooser<Command> autoChooser = new SendableChooser<>();

    public void AllianceColor(){
        allianceColor.setDefaultOption("Non-Event", "orange");
            allianceColor.addOption("Red Alliance", "red");
            allianceColor.addOption("Blue Alliance", "blue");
        
        SmartDashboard.putData(allianceColor);
    }

    public void AutoChooser(){
        autoChooser.setDefaultOption("No Auto", new noAuto());
            autoChooser.addOption("Shoot Only", new shootOnly());

        SmartDashboard.putData(autoChooser);
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

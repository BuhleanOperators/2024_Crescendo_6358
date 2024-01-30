// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class intakeSystem extends SubsystemBase {
  /** Creates a new intakeSystem. */
  //! Assumed to be SparkMaxes Not for sure
  private CANSparkMax rightIntake;
  private CANSparkMax leftIntake;

  public intakeSystem() {
    rightIntake = new CANSparkMax(IntakeConstants.rightIntakeID, MotorType.kBrushless);
    rightIntake.setInverted(IntakeConstants.rightIntakeInverted);
    rightIntake.setSmartCurrentLimit(IntakeConstants.smartCurrentLimit);
    rightIntake.setIdleMode(IntakeConstants.idleMode);
    rightIntake.burnFlash();

    leftIntake = new CANSparkMax(IntakeConstants.leftIntakeID, MotorType.kBrushless);
    leftIntake.setInverted(IntakeConstants.leftIntakeInverted);
    leftIntake.setSmartCurrentLimit(IntakeConstants.smartCurrentLimit);
    leftIntake.setIdleMode(IntakeConstants.idleMode);
    leftIntake.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeeds(double speed){
    rightIntake.set(speed);
    leftIntake.set(speed);
  }
}

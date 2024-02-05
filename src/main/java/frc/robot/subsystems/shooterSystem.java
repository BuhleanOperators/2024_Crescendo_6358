// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonInclude.Include;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class shooterSystem extends SubsystemBase {
  /** Creates a new intake. */
  private CANSparkMax rightMotor;
  private CANSparkMax leftMotor;
  private RelativeEncoder rightEncoder;

  public shooterSystem() {
    rightMotor = new CANSparkMax(ShooterConstants.rightMotorID, MotorType.kBrushless);
    rightMotor.setSmartCurrentLimit(ShooterConstants.currentLimit);
    rightMotor.setInverted(ShooterConstants.setRightInverted);
    rightMotor.setIdleMode(ShooterConstants.idleMode);
    rightMotor.burnFlash();

    leftMotor = new CANSparkMax(ShooterConstants.leftMotorID, MotorType.kBrushless);
    leftMotor.setSmartCurrentLimit(ShooterConstants.currentLimit);
    leftMotor.setInverted(ShooterConstants.setLeftInverted);
    leftMotor.setIdleMode(ShooterConstants.idleMode);
    leftMotor.burnFlash();

    rightEncoder = rightMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //^ Accsessor Methods
  public double getShooterRPM(){
    return rightEncoder.getVelocity();
  }
  //^ Void Methods
  public void setSpeed(double speed){
    rightMotor.set(speed);
    leftMotor.set(speed);
  }
}

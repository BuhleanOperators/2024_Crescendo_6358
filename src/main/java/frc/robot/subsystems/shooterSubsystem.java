// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class shooterSubsystem extends SubsystemBase {
  //^ Initialization and methods for the shooter system
  private CANSparkMax rightMotor;
  private CANSparkMax leftMotor;
  private RelativeEncoder rightEncoder;

  public shooterSubsystem() {
    rightMotor = new CANSparkMax(ShooterConstants.rightMotorID, MotorType.kBrushless); //Create the SPARK Max
    rightMotor.setSmartCurrentLimit(ShooterConstants.currentLimit); //Set the current limit
    rightMotor.setInverted(ShooterConstants.setRightInverted); //Set inverted if needed
    rightMotor.setIdleMode(ShooterConstants.idleMode); //Set the idle mode of the motors
    rightMotor.burnFlash(); //Save all changes made

    leftMotor = new CANSparkMax(ShooterConstants.leftMotorID, MotorType.kBrushless);
    leftMotor.setSmartCurrentLimit(ShooterConstants.currentLimit);
    leftMotor.setInverted(ShooterConstants.setLeftInverted);
    leftMotor.setIdleMode(ShooterConstants.idleMode);
    leftMotor.burnFlash();

    rightEncoder = rightMotor.getEncoder(); //Get the inbuilt encoder
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
    //Set the speed of the shooter
    rightMotor.set(speed);
    leftMotor.set(speed);
  }
}

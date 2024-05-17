// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class driveTrain extends SubsystemBase {
  //^ Initialization and methods for the drive base
  private CANSparkMax rightLead;
  private CANSparkMax rightFollow;
  private CANSparkMax leftLead;
  private CANSparkMax leftFollow;
  private SparkPIDController rightPID;
  private SparkPIDController leftPID;
  private static RelativeEncoder rightEncoder;
  private static RelativeEncoder leftEncoder;
  private DifferentialDriveKinematics m_kinematics;
  private ADIS16448_IMU m_gyro;

  public driveTrain() {
    //~ Configure SparkMaxs
    rightLead = new CANSparkMax(DriveConstants.rightLeadID, MotorType.kBrushless); //Create the SPARK Max
    rightLead.setSmartCurrentLimit(DriveConstants.smartCurrentLimit); //Set the current limit of the SPARK Max
    rightLead.setInverted(DriveConstants.rightLeadInvert); //Invert the motor contoller if needed
    rightLead.burnFlash(); //Save all changes made

    rightFollow = new CANSparkMax(DriveConstants.rightFollowID, MotorType.kBrushless);
    rightFollow.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);
    rightFollow.setInverted(DriveConstants.rightFollowInvert);
    rightFollow.follow(rightLead);
    rightFollow.burnFlash();

    leftLead = new CANSparkMax(DriveConstants.leftLeadID, MotorType.kBrushless);
    leftLead.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);
    leftLead.setInverted(DriveConstants.leftLeadInvert);
    leftLead.burnFlash();

    leftFollow = new CANSparkMax(DriveConstants.leftFollowID, MotorType.kBrushless);
    leftFollow.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);
    leftFollow.setInverted(DriveConstants.leftFollowInvert);
    leftFollow.follow(leftLead);
    leftFollow.burnFlash();

    //& Configure Encoders
    rightEncoder = rightLead.getEncoder(); //Get the inbuilt encoder from the motor
    leftEncoder = leftLead.getEncoder();

    //* Configure PID controllers
    rightPID = rightLead.getPIDController(); //Get the inbuilt PID controller
    rightPID.setP(DriveConstants.rightP); //Set the P value
    rightPID.setI(DriveConstants.rightI); //Set the I value
    rightPID.setD(DriveConstants.rightD); //Set the D value
    rightPID.setFF(DriveConstants.rightFF); //Set the FF (feedforward) value
    rightPID.setFeedbackDevice(rightEncoder); //Set the feedback device to the encoder
    
    leftPID = leftLead.getPIDController();
    leftPID.setP(DriveConstants.leftP);
    leftPID.setI(DriveConstants.leftI);
    leftPID.setD(DriveConstants.leftD);
    leftPID.setFF(DriveConstants.leftFF);
    leftPID.setFeedbackDevice(leftEncoder);

    //? Configure Kinematics
    m_kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth); //Set the trackwidth of the robot
    m_gyro = new ADIS16448_IMU(); //Create the gyro
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
//^ Accsessor Methods
  public RelativeEncoder getRightEncoder(){
    return rightEncoder;
  }
  public RelativeEncoder getLeftEncoder(){
    return leftEncoder;
  }
  public double getAverageDistance(){
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
  }
  public double getAngle(){
    return m_gyro.getGyroAngleZ();
  }

//^ Void Methods
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
    //Set the PID reference to a speed
    rightPID.setReference(speeds.rightMetersPerSecond, CANSparkBase.ControlType.kVoltage);
    leftPID.setReference(speeds.leftMetersPerSecond, CANSparkBase.ControlType.kVoltage);
  }
  public void arcadeDrive(double xSpeed, double rot){
    //Set the speeds of the drive train 
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }
  public void setAutoSpeeds(double distance){
    //Set the desired distance to drive forward
    rightPID.setReference(-distance, CANSparkBase.ControlType.kVoltage);
    leftPID.setReference(-distance, CANSparkBase.ControlType.kVoltage);
  }
  public void setReverseAutoSpeeds(double posDistance){
    //Set the desired distanc eto drive in reverse
    rightPID.setReference(posDistance, CANSparkBase.ControlType.kVoltage);
    leftPID.setReference(posDistance, CANSparkBase.ControlType.kVoltage);
  }
  public void setAutoTurnSpeeds(double speed){
    //Set the speed of the drive train to turn
    rightPID.setReference(speed, CANSparkBase.ControlType.kVoltage);
    leftPID.setReference(-speed, CANSparkBase.ControlType.kVoltage);
  }
  public void stop(){
    //Stop the drive train
    rightLead.set(0);
    leftLead.set(0);
  }
  public void resetGyro(){
    //Reset the gyro
    m_gyro.reset();
  }
  public void calibrateGyro(){
    //Calibrate the gyro
    m_gyro.calibrate();
  }
  public void setIdleMode(CANSparkBase.IdleMode mode){
    //Set the idle mode of all four drive motors
    rightLead.setIdleMode(mode);
    rightFollow.setIdleMode(mode);
    leftLead.setIdleMode(mode);
    leftFollow.setIdleMode(mode);
  }
  public void resetEncoders(){
    //Reset the encoder values
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }
}

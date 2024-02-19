// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class driveTrain extends SubsystemBase {
  /** Creates a new driveTrain. */
  private CANSparkMax rightLead;
  private CANSparkMax rightFollow;
  private CANSparkMax leftLead;
  private CANSparkMax leftFollow;
  private SparkPIDController rightPID;
  private SparkPIDController leftPID;
  private static RelativeEncoder rightEncoder;
  private static RelativeEncoder leftEncoder;
  private DifferentialDriveKinematics m_kinematics;
  private DifferentialDrive m_Drive;
  private ADIS16448_IMU m_gyro;

  public driveTrain() {
    //~ Configure SparkMaxs
    rightLead = new CANSparkMax(DriveConstants.rightLeadID, MotorType.kBrushless);
    // rightLead.restoreFactoryDefaults();
    rightLead.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);
    rightLead.setInverted(DriveConstants.rightLeadInvert);
    rightLead.setIdleMode(DriveConstants.idleMode);
    rightLead.burnFlash();

    rightFollow = new CANSparkMax(DriveConstants.rightFollowID, MotorType.kBrushless);
    // rightFollow.restoreFactoryDefaults();
    rightFollow.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);
    rightFollow.setInverted(DriveConstants.rightFollowInvert);
    rightFollow.setIdleMode(DriveConstants.idleMode);
    rightFollow.follow(rightLead);
    rightFollow.burnFlash();

    leftLead = new CANSparkMax(DriveConstants.leftLeadID, MotorType.kBrushless);
    // leftLead.restoreFactoryDefaults();
    leftLead.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);
    leftLead.setInverted(DriveConstants.leftLeadInvert);
    leftLead.setIdleMode(DriveConstants.idleMode);
    leftLead.burnFlash();

    leftFollow = new CANSparkMax(DriveConstants.leftFollowID, MotorType.kBrushless);
    // leftFollow.restoreFactoryDefaults();
    leftFollow.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);
    leftFollow.setInverted(DriveConstants.leftFollowInvert);
    leftFollow.setIdleMode(DriveConstants.idleMode);
    leftFollow.follow(leftLead);
    leftFollow.burnFlash();

    //& Configure Encoders
    //? Default Constructor instead?
    rightEncoder = rightLead.getEncoder();
   // rightEncoder.setInverted(true);
    // rightEncoder.setPositionConversionFactor(-4);
    // rightLead.burnFlash();

    leftEncoder = leftLead.getEncoder();
    // leftEncoder.setPositionConversionFactor(1);
    // leftLead.burnFlash();
    // rightEncoder = rightLead.getEncoder(com.revrobotics.SparkRelativeEncoder.Type.kHallSensor, DriveConstants.countsPerRev);

    // leftEncoder = leftLead.getEncoder(com.revrobotics.SparkRelativeEncoder.Type.kHallSensor, DriveConstants.countsPerRev);

    //* Configure PID controllers
    rightPID = rightLead.getPIDController();
    rightPID.setP(DriveConstants.rightP);
    rightPID.setI(DriveConstants.rightI);
    rightPID.setD(DriveConstants.rightD);
    rightPID.setFF(DriveConstants.rightFF);
    rightPID.setFeedbackDevice(rightEncoder);
    //^ Smart Motion Values
    //rightPID.setSmartMotionMaxVelocity(DriveConstants.maxSpeed, DriveConstants.slotID);
    
    leftPID = leftLead.getPIDController();
    leftPID.setP(DriveConstants.leftP);
    leftPID.setI(DriveConstants.leftI);
    leftPID.setD(DriveConstants.leftD);
    leftPID.setFF(DriveConstants.leftFF);
    leftPID.setFeedbackDevice(leftEncoder);
    //^ Smart Motion Values
    //leftPID.setSmartMotionMaxVelocity(DriveConstants.maxSpeed, DriveConstants.slotID);

    //? Configure Kinematics
    m_kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);

    m_Drive = new DifferentialDrive(leftLead, rightLead);
    m_Drive.setSafetyEnabled(false);
    m_gyro = new ADIS16448_IMU();
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
  public static void resetEncoders(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }
  public double getRightEncoderDistance(){
    return rightEncoder.getPosition();
  }
  public double getLeftEncoderDistance(){
    return leftEncoder.getPosition();
  }
  public double getAverageDistance(){
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
  }
  public double getAngle(){
    return m_gyro.getGyroAngleZ();
  }
  public void resetGyro(){
    m_gyro.reset();
  }

//^ Drive Methods
//?Is this the best way to do this?
  public void setDriveSpeeds(DifferentialDriveWheelSpeeds speeds){
    rightPID.setReference(speeds.rightMetersPerSecond, CANSparkBase.ControlType.kVoltage);
    leftPID.setReference(speeds.leftMetersPerSecond, CANSparkBase.ControlType.kVoltage);
  }
  public void arcadeDrive(double xSpeed, double rot){
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setDriveSpeeds(wheelSpeeds);
  }
  public void setAutoSpeeds(double distance){
    //? voltage or duty cycle
    rightPID.setReference(distance, CANSparkBase.ControlType.kDutyCycle);
    leftPID.setReference(distance, CANSparkBase.ControlType.kDutyCycle);
  }
  public void setAutoTurnSpeeds(double speeds){
    rightPID.setReference(speeds, CANSparkBase.ControlType.kVoltage);
    leftPID.setReference(-speeds, CANSparkBase.ControlType.kVoltage);
  }
  public void newDrive(double xSpeed, double rot){
    m_Drive.arcadeDrive(xSpeed, rot);
  }
  public void stop(){
    rightLead.set(0);
    leftLead.set(0);
  }
  // public void arcadeDrive(double xSpeed, double rot){
  //   DifferentialDrive.arcadeDriveIK(xSpeed, rot, false);
  // }
  // public void tankDrive(double right, double left){
  //   var leftSpeed = leftPID.setReference(left, CANSparkBase.ControlType.kVelocity);
  //   var rightSpeed = rightPID.setReference(right, CANSparkBase.ControlType.kVelocity);
  //   DifferentialDrive.tankDriveIK(leftSpeed, rightSpeed, false);
  // }
}

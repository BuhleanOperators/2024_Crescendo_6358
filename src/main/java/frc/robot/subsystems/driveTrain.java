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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
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
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private DifferentialDriveKinematics m_kinematics;
  private DifferentialDriveOdometry m_odomotry;
  private ADIS16448_IMU m_gyro;

  public driveTrain() {
    //~ Configure SparkMaxs
    rightLead = new CANSparkMax(DriveConstants.rightLeadID, MotorType.kBrushless);
    rightLead.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);
    rightLead.setInverted(DriveConstants.rightLeadInvert);
    rightLead.setIdleMode(DriveConstants.idleMode);
    rightLead.burnFlash();

    rightFollow = new CANSparkMax(DriveConstants.rightFollowID, MotorType.kBrushless);
    rightFollow.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);
    rightFollow.setInverted(DriveConstants.rightFollowInvert);
    rightFollow.setIdleMode(DriveConstants.idleMode);
    rightFollow.follow(rightLead);
    rightFollow.burnFlash();

    leftLead = new CANSparkMax(DriveConstants.leftLeadID, MotorType.kBrushless);
    leftLead.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);
    leftLead.setInverted(DriveConstants.leftLeadInvert);
    leftLead.setIdleMode(DriveConstants.idleMode);
    leftLead.burnFlash();

    leftFollow = new CANSparkMax(DriveConstants.leftFollowID, MotorType.kBrushless);
    leftFollow.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);
    leftFollow.setInverted(DriveConstants.leftFollowInvert);
    leftFollow.setIdleMode(DriveConstants.idleMode);
    leftFollow.follow(leftLead);
    leftFollow.burnFlash();

    //& Configure Encoders
    rightEncoder = rightLead.getEncoder();
    leftEncoder = leftLead.getEncoder();
  
    //* Configure PID controllers
    rightPID = rightLead.getPIDController();
    rightPID.setP(DriveConstants.rightP);
    rightPID.setI(DriveConstants.rightI);
    rightPID.setD(DriveConstants.rightD);
    rightPID.setFF(DriveConstants.rightFF);
    rightPID.setFeedbackDevice(rightEncoder);
    
    leftPID = leftLead.getPIDController();
    leftPID.setP(DriveConstants.leftP);
    leftPID.setI(DriveConstants.leftI);
    leftPID.setD(DriveConstants.leftD);
    leftPID.setFF(DriveConstants.leftFF);
    leftPID.setFeedbackDevice(leftEncoder);

    //? Configure Kinematics
    m_kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);

    // Configure Gyro
    m_gyro = new ADIS16448_IMU();

    //! Odomotry
    m_odomotry = new DifferentialDriveOdometry(new Rotation2d(m_gyro.getGyroAngleX()), 0, 0);
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
  public ADIS16448_IMU getGyro(){
    return m_gyro;
  }
  public double getHeading(){
    return Math.IEEEremainder(m_gyro.getGyroAngleX(), 360);
  }
  public void resetHeading(){
    m_gyro.reset();
  }
  public Command resetHeadingCommand(){
    return runOnce(
      () -> {
        m_gyro.reset();
      }
    );
  }
  public double getTurnRate(){
    return m_gyro.getRate();
  }

//^ Drive Methods
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
    rightPID.setReference(speeds.rightMetersPerSecond, CANSparkBase.ControlType.kVoltage);
    leftPID.setReference(speeds.leftMetersPerSecond, CANSparkBase.ControlType.kVoltage);
  }
  public void arcadeDrive(double xSpeed, double rot){
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }
//^ Odomotry Methods
  public void updateOdomotry(){
    m_odomotry.update(new Rotation2d(m_gyro.getGyroAngleX()), getLeftEncoder().getPosition(), getRightEncoder().getPosition());
  }
}

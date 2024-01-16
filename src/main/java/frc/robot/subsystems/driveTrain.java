// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class driveTrain extends SubsystemBase {
  /** Creates a new driveTrain. */
  private CANSparkMax rightLead;
  private CANSparkMax rightFollow;
  private CANSparkMax leftLead;
  private CANSparkMax leftFollow;

  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDriveOdometry m_odometry;
  private final SimpleMotorFeedforward m_feedForward;

  private final PIDController leftPidController;
  private final PIDController rightPidController;
//TODO figure out hall effect sensors
  private static Encoder rightDriveEncoder;
  private static Encoder leftDriveEncoder;

  private ADIS16448_IMU m_gyro;
  public driveTrain() {
    rightLead = new CANSparkMax(Constants.DriveConstants.rightLeadID, MotorType.kBrushless);
    rightFollow = new CANSparkMax(Constants.DriveConstants.rightFollowID, MotorType.kBrushless);
    rightFollow.follow(rightLead);

    leftLead = new CANSparkMax(Constants.DriveConstants.leftLeadID, MotorType.kBrushless);
    leftFollow = new CANSparkMax(Constants.DriveConstants.leftFollowID, MotorType.kBrushless);
    leftFollow.follow(leftLead);

    leftPidController = new PIDController(Constants.DriveConstants.lP, Constants.DriveConstants.lI, Constants.DriveConstants.lD);
    rightPidController = new PIDController(Constants.DriveConstants.rP, Constants.DriveConstants.rI, Constants.DriveConstants.rD);
    
    rightDriveEncoder = new Encoder(Constants.DriveConstants.rightDriveEncoderPort1, Constants.DriveConstants.rightDriveEncoderPort2);
    leftDriveEncoder = new Encoder(Constants.DriveConstants.leftDriveEncoderPort1, Constants.DriveConstants.leftDriveEncoderPort2);

    m_gyro = new ADIS16448_IMU();

    m_kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.trackWidth);
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(m_gyro.getGyroAngleX()), leftEncoderDistance(), rightEncoderDistance());
    m_feedForward = new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static double rightEncoderDistance(){
    return rightDriveEncoder.getDistance();
  }

  public static double leftEncoderDistance(){
    return leftDriveEncoder.getDistance();
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
    final double leftFeedForward = m_feedForward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedForward = m_feedForward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = leftPidController.calculate(leftDriveEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = rightPidController.calculate(rightDriveEncoder.getRate(), speeds.rightMetersPerSecond);

    leftLead.setVoltage(leftOutput + leftFeedForward);
    rightLead.setVoltage(rightOutput + rightFeedForward);
  }

  public void drive(double xSpeed, double rot){
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public void updateOdometry(){
    m_odometry.update(new Rotation2d(m_gyro.getGyroAngleX()), leftEncoderDistance(), rightEncoderDistance());
  }
}

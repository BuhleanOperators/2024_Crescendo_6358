// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class DriveIOSparkMax implements DriveIO{

    private final CANSparkMax leftLeader = new CANSparkMax(DriveConstants.leftLeadID, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(DriveConstants.rightLeadID, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(DriveConstants.leftFollowID, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(DriveConstants.rightFollowID, MotorType.kBrushless);
    
    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
    private final SparkPIDController leftPID = leftLeader.getPIDController();
    private final SparkPIDController rightPID = rightLeader.getPIDController();

    private final ADIS16448_IMU gyro = new ADIS16448_IMU();

    public DriveIOSparkMax() {
        leftLeader.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        leftLeader.setCANTimeout(250);
        rightLeader.setCANTimeout(250);
        leftFollower.setCANTimeout(250);
        rightFollower.setCANTimeout(250);

        leftLeader.setInverted(DriveConstants.invertLeft);
        rightLeader.setInverted(DriveConstants.invertRight);
        leftFollower.follow(leftLeader, false);
        rightFollower.follow(rightLeader, false);

        leftLeader.enableVoltageCompensation(12.0);
        rightLeader.enableVoltageCompensation(12.0);
        leftLeader.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);
        rightLeader.setSmartCurrentLimit(DriveConstants.smartCurrentLimit);

        leftPID.setP(DriveConstants.leftP);
        leftPID.setD(DriveConstants.leftD);
        rightPID.setP(DriveConstants.rightP);
        rightPID.setD(DriveConstants.rightD);

        leftLeader.burnFlash();
        rightLeader.burnFlash();
        leftFollower.burnFlash();
        rightFollower.burnFlash();
    }

    @Override
    public void updateInputs(DriveIOInputs inputs){
        inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() / DriveConstants.gearRatio);
        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / DriveConstants.gearRatio);
        inputs.leftAppliedVolts = leftLeader.getAppliedOutput() * leftLeader.getBusVoltage();
        inputs.leftCurrentAmps = new double[] {leftLeader.getOutputCurrent(), leftFollower.getOutputCurrent()};

        inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition() / DriveConstants.gearRatio);
        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / DriveConstants.gearRatio);
        inputs.rightAppliedVolts = rightLeader.getAppliedOutput() * rightLeader.getBusVoltage();
        inputs.rightCurrentAmps = new double[] {rightLeader.getOutputCurrent(), rightFollower.getOutputCurrent()};

        inputs.gyroYaw = Rotation2d.fromDegrees(gyro.getGyroAngleZ());
        inputs.idleMode = leftLeader.getIdleMode();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts){
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }

    @Override
    public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts){
        leftPID.setReference(
            Units.radiansPerSecondToRotationsPerMinute(leftRadPerSec * DriveConstants.gearRatio),
            ControlType.kVelocity,
            0,
            leftFFVolts);

        rightPID.setReference(
            Units.radiansPerSecondToRotationsPerMinute(rightRadPerSec * DriveConstants.gearRatio),
            ControlType.kVelocity,
            0,
            rightFFVolts);
    }

    @Override
    public void setIdleMode(CANSparkBase.IdleMode idleMode){
        leftLeader.setIdleMode(idleMode);
        rightLeader.setIdleMode(idleMode);
        leftFollower.setIdleMode(idleMode);
        rightFollower.setIdleMode(idleMode);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class ShooterIoSparkMax implements ShooterIO{

    private final CANSparkMax leader = new CANSparkMax(ShooterConstants.leadID, MotorType.kBrushless);
    private final CANSparkMax follower = new CANSparkMax(ShooterConstants.followID, MotorType.kBrushless);
    private final RelativeEncoder encoder = leader.getEncoder();
    private final SparkPIDController pid = leader.getPIDController();

    public ShooterIoSparkMax() {
        leader.restoreFactoryDefaults();
        follower.restoreFactoryDefaults();

        leader.setCANTimeout(250);
        follower.setCANTimeout(250);

        leader.setInverted(true);
        follower.follow(leader, true);

        leader.enableVoltageCompensation(12.0);
        leader.setSmartCurrentLimit(ShooterConstants.currentLimit);

        leader.burnFlash();
        follower.burnFlash();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.positionRad = Units.rotationsToRadians(encoder.getPosition());
        inputs.VelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        inputs.AppliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
        inputs.CurrentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    }

    @Override
    public void setVoltage(double volts){
        leader.setVoltage(volts);
    }

    @Override
    public void setVelocity(double velocityRadPerSec, double ffVolts){
        pid.setReference(
            Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec), 
            ControlType.kVelocity,
            0,
            ffVolts,
            ArbFFUnits.kVoltage);
    }

    @Override
    public void stop(){
        leader.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD){
        pid.setP(kP, 0);
        pid.setI(kI, 0);
        pid.setD(kD, 0);
        pid.setFF(0, 0);
    }
}

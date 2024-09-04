// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double[] leftCurrentAmps = new double[] {};

        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double[] rightCurrentAmps = new double[] {};

        public Rotation2d gyroYaw = new Rotation2d();
        public CANSparkBase.IdleMode idleMode = CANSparkBase.IdleMode.kCoast;
    }

    //**Updates the set of updatable inputs */
    public default void updateInputs (DriveIOInputs inputs) {}

    //**Run open loop at specified voltage */
    public default void setVoltage (double leftVolts, double rightVolts) {}

    //**Run closed loop at specififed velocity */
    public default void setVelocity (double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {}

    //**Set the idle mode of motors */
    public default void setIdleMode (CANSparkBase.IdleMode idleMode){}
} 

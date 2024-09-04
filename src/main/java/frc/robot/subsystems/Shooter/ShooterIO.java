// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;


/** Add your docs here. */
public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs{
        public double positionRad = 0.0;
        public double VelocityRadPerSec = 0.0;
        public double AppliedVolts = 0.0;
        public double[] CurrentAmps = new double[] {};
    }
    //** Updates the set of loggable inputs */
    public default void updateInputs(ShooterIOInputs inputs){}

    //** Run open loop at specified voltage */
    public default void setVoltage(double volts){}

    //** Run closed loop at specified velocity */
    public default void setVelocity(double VelocityRadPerSec, double ffVolts){}

    //** Stop in open loop */
    public default void stop(){}

    //** Set velocity PID constants */
    public default void configurePID(double kP, double kI, double kD){}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FlywheelsIO {
    @AutoLog
    public static class FlywheelsIOInputs{}

    //**Update list of updatable inputs */
    public default void updateInputs(FlywheelsIOInputs inputs){}

    //**Set the motor to a specified speed */
    public default void setSpeed(double speed){}

    //**Stop the motor */
    public default void stop(){}
}

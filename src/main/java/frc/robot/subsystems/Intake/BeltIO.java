// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface BeltIO {

    @AutoLog
    public static class BeltIoInputs {
        public double AppliedVolts = 0.0;
        public double[] CurrentAmps = new double[] {};
    }

    //**Updates set up updatable inputs */
    public default void updateInputs(BeltIoInputs inputs){}

    //**Sets the motor to specified speed (-1 to 1) */
    public default void setSpeed(double speed){}

    //**Stops the motor */
    public default void stop(){}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class DriveConstants {
    //TODO Set all values
    //!Odomotry Values
    public static final double trackWidth = 0; //meters
    public static final double wheelRadius = 0; //meters
    public static final double maxSpeed = 0; //meters per second
    public static final double maxAngularSpeed = 2 * Math.PI; //one rotation per second

    public static final int rightLeadID = 0;
    public static final int rightFollowID = 0;
    public static final int leftLeadID = 0;
    public static final int leftFollowID = 0;

    public static final int rightDriveEncoderPort1 = 0;
    public static final int rightDriveEncoderPort2 = 0;
    public static final int leftDriveEncoderPort1 = 0;
    public static final int leftDriveEncoderPort2 = 0;
    //& Encoder Values
    //TODO Figure out how to use NEO's hall effect sensors
    public static final double encoderResolution = 0;
    public static final double DistancePerPulse = 2 * Math.PI * wheelRadius / encoderResolution;
    //* PID Controller Values
    public static final double lP = 0;
    public static final double lI = 0;
    public static final double lD = 0;
    public static final double rP = 0;
    public static final double rI = 0;
    public static final double rD = 0;
    //^ FeedForward Values
    //~ kS = 0.5 on tile and 0.95 on carpet
    public static final double kS = 0;
    public static final double kV = 0;
  }
}

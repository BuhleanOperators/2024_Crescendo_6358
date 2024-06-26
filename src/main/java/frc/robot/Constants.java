// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;

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
    public static final int kCoPilotControllerPort = 1;
    public static final double deadbandCutoffDrive = 0.2;
    public static final double deadbandCutoffRot = 0.5;

    public static final int intakeIn = 6;
    public static final int intakeOut = 5;

    public static final int BUTTON_shooterSpeaker = 6;
    public static final int BUTTON_shooterAmp = 2;
    public static final int BUTTON_shooterAmpSlow = 3;
    public static final int BUTTON_beltsOut = 5;
    public static final int BUTTON_extendPiston = 1;
    public static final int BUTTON_retractPiston = 2;

  }
  public static class DriveConstants{
    //! Odomotry Values
    public static final double trackWidth = Units.inchesToMeters(22.5); //meters
    public static final double wheelRadius = Units.inchesToMeters(3); //meters
    public static final double maxSpeed = 6; //volts per second
    public static final double maxAngularSpeed = Math.PI * maxSpeed; //one rotation per second
    public static final int slotID = 0;

    //~ Drive Values
    public static final int rightLeadID = 1; //Bolt: 7
    public static final int rightFollowID = 2; //Bolt: 4
    public static final int leftLeadID = 3; //Bolt 3
    public static final int leftFollowID = 4; //Bolt: 9

    public static final int smartCurrentLimit = 40; //amps
    public static final boolean rightLeadInvert = true;
    public static final boolean rightFollowInvert = true;
    public static final boolean leftLeadInvert = false;
    public static final boolean leftFollowInvert = false;
    public static final CANSparkBase.IdleMode idleModeTeleop = IdleMode.kCoast;
    public static final CANSparkBase.IdleMode idleModeAuto = IdleMode.kBrake;
    public static final CANSparkBase.IdleMode idleModeDisabled = IdleMode.kCoast;

    //& Encoder Values
    public static final int countsPerRev = 42;
    
    //* PID Controller Values
    //FF values are feedForwards
    //FF = 0.5 on tile and 0.95 on carpet
    public static final double rightP = 0.15;
    public static final double rightI = 0;
    public static final double rightD = 0;
    public static final double rightFF = 0.95;

    public static final double leftP = 0.15;
    public static final double leftI = 0;
    public static final double leftD = 0;
    public static final double leftFF = 0.95;
  }
  public static class ShooterConstants{
    public static final int rightMotorID = 5;
    public static final int leftMotorID = 6;

    public static final int currentLimit = 40;
    public static final boolean setRightInverted = true;
    public static final boolean setLeftInverted = false;
    public static final IdleMode idleMode = IdleMode.kCoast;

    public static final double speakerSpeed = 0.90;
    public static final double ampSpeed = 0.35;
    public static final double ampSpeedSlow = 0.2;

    public static final double speakerRPM = 4100;
    public static final double ampRPM = 1900;
  }
  public static class PneumaticsConstants {
    public static final int PISTON_RIGHT_FORWARD = 0;
    public static final int PISTON_RIGHT_BACK = 1;
    public static final int PISTON_LEFT_FORWARD = 2;
    public static final int PISTON_LEFT_BACK = 3;
  }
  public static class IntakeConstants{
    public static final int intakeID = 1;
    public static final int beltID = 0;
  }
  public static class LedConstants{
    public static final int port = 6;
    public static final int length = 30;
  }
}

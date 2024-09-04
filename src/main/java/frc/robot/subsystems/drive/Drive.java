// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final DifferentialDriveOdometry odometry = 
    new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV);
  private final SysIdRoutine sysId;

  public Drive(DriveIO io) {
    this.io = io;

    sysId = 
      new SysIdRoutine(new SysIdRoutine.Config(
        null,
        null,
        null,
        (state) -> Logger.recordOutput("Drive/SysIDState", state.toString())
      ), 
      new SysIdRoutine.Mechanism(
        (voltage) -> driveVolts(voltage.in(Volts), voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    //Update odometry
    odometry.update(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters());
  }

  //**Run open loop at specified voltage */
  public void driveVolts(double leftVolts, double rightVolts){
    io.setVoltage(leftVolts, rightVolts);
  }

  //**Run closed loop at specified velocity */
  public void driveVelocity(double leftMetersPerSec, double rightMetersPerSec){
    Logger.recordOutput("Drive/LeftVelocitySetpointMetersPerSec", leftMetersPerSec);
    Logger.recordOutput("Drive/RightVelocitySetpointMetersPerSec", rightMetersPerSec);

    double leftRadPerSec = leftMetersPerSec / DriveConstants.wheelRadius;
    double rightRadPerSec = rightMetersPerSec / DriveConstants.wheelRadius;

    io.setVelocity(
      leftRadPerSec,
      rightRadPerSec,
      feedForward.calculate(leftMetersPerSec),
      feedForward.calculate(rightRadPerSec));
  }

  //**Run open loop based on joystick positions */
  public void arcadeDrive(double xSpeed, double zRotation){
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    io.setVoltage(speeds.left * 12.0, speeds.right * 12.0); //. * 12.0 because thats the max voltage possible
  }

  //**Stops the drive */
  public void stop(){
    io.setVoltage(0.0, 0.0);
  }

  //**Returns a command to run a quasi-static test in the specified direction */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return sysId.quasistatic(direction);
  }

  //**Returns a command to run a dynamic test in the specified direction */
  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return sysId.dynamic(direction);
  }

  //**Returns current odometry pose */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  //**Resets current odometry pose */
  public void setPose(Pose2d pose){
    odometry.resetPosition(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters(), pose);
  }

  //**Returns the position of the left wheels in meters */
  @AutoLogOutput
  public double getLeftPositionMeters(){
    return inputs.leftPositionRad * DriveConstants.wheelRadius;
  }
  //**Returns the position of the right wheels in meters */
  @AutoLogOutput
  public double getRightPositionMeters(){
    return inputs.rightPositionRad * DriveConstants.wheelRadius;
  }
  //**Returns the velocity of the left wheels in meters/sec */
  @AutoLogOutput
  public double getLeftVelocityMetersPerSec(){
    return inputs.leftVelocityRadPerSec * DriveConstants.wheelRadius;
  }
  //**Returns the velocity of the right wheels in meters/sec */
  @AutoLogOutput
  public double getRightVelocityMetersPerSec(){
    return inputs.rightVelocityRadPerSec * DriveConstants.wheelRadius;
  }

  //**Returns the average velocity in rad/sec */
  public double getAverageVelocity(){
    return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0;
  }

  //**Sets the idle mode of the drive motors */
  public void setIdleMode(CANSparkBase.IdleMode idleMode){
    io.setIdleMode(idleMode);
  }
}

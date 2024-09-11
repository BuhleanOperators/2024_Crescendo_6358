// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;
  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;

    switch (Constants.currentMode){
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0, 0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
    }

    //Configure SysId
    sysId = 
      new SysIdRoutine(
        new SysIdRoutine.Config(
          null, null, null, //Default config
          (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    
    Logger.processInputs("Shooter", inputs);
  }

  //**Run open loop at specified voltage */
  public void runVolts(double volts){
    io.setVoltage(volts);
  }
  
  //**Run closed loop at specified velocity */
  public void runVelocity(double velocityRPM){
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRPM, velocityRPM);

    Logger.recordOutput("Shooter/SetpointRPM", velocityRPM);
  }

  //**Stop the shooter */
  public void stop(){
    io.stop();
  }

  //**Return a command to run a quasistaitc test in the specified direction */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return sysId.quasistatic(direction);
  }

  //**Return a command to run a dynamic test in the specified direction */
  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return sysIdDynamic(direction);
  }

  //**Returns current velocity in RPM */
  public double getShooterRPM(){
    return Units.radiansPerSecondToRotationsPerMinute(inputs.VelocityRadPerSec);
  }

  //**Returns current velocity in radians per second */
  public double getCharacterizationVelocity(){
    return inputs.VelocityRadPerSec;
  }

  public double getVelocityRPM(){
    return io.getShooterRPM();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PhaseTwoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.Constants.DriveConstants;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.phase2;
// import frc.robot.commands.arcadeDrive;
import frc.robot.commands.shooterRun;
import frc.robot.commands.Pneumatics.ToggleSolenoid;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.phaseTwoSystem;
import frc.robot.subsystems.pneumatics;
// import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.shooterSystem;

import java.lang.management.OperatingSystemMXBean;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final driveTrain m_DriveTrain = new driveTrain();
  private final shooterSystem m_ShooterSystem = new shooterSystem();
  private final pneumatics m_pneumatics = new pneumatics();
  private final phaseTwoSystem m_PhaseTwoSystem = new phaseTwoSystem();

  private double deadbandreturn;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController xDriver = new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    smartDashboard();
    
    // m_DriveTrain.setDefaultCommand(new arcadeDrive(() -> deadband(getXDriver().getLeftY() * Constants.DriveConstants.maxSpeed, OperatorConstants.deadbandCutoffDrive), () -> deadband(getXDriver().getRightX() * Constants.DriveConstants.maxAngularSpeed, OperatorConstants.deadbandCutoffRot), m_DriveTrain));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    //new JoystickButton(xDriver, OperatorConstants.intakeButton).toggleOnTrue(new shooterRun(ShooterConstants.speed, m_ShooterSystem));
    new JoystickButton(xDriver, OperatorConstants.BUTTON_shooterSpeaker).onTrue(new shooterRun(ShooterConstants.speakerSpeed, m_ShooterSystem))
      .onFalse(new shooterRun(0, m_ShooterSystem));
    new JoystickButton(xDriver, OperatorConstants.BUTTON_shooterAmp).onTrue(new shooterRun(ShooterConstants.ampSpeed, m_ShooterSystem))
      .onFalse(new shooterRun(0, m_ShooterSystem));

    new JoystickButton(xDriver, OperatorConstants.BUTTON_belts).onTrue(new phase2(PhaseTwoConstants.speed, m_PhaseTwoSystem))
      .onFalse(new phase2(0, m_PhaseTwoSystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  // new JoystickButton(xDriver, OperatorConstants.BUTTON_togglePiston).onTrue(new ToggleSolenoid(m_pneumatics.getFirstSolenoid(), m_pneumatics));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  private void smartDashboard(){

  }

  public XboxController getXDriver(){
    return xDriver;
  }

  private double deadband(double JoystickValue, double DeadbandCutOff){
    if (JoystickValue < DeadbandCutOff && JoystickValue > (DeadbandCutOff * (-1))) {
      deadbandreturn = 0;
      }
      else {
      deadbandreturn = (JoystickValue - (Math.abs(JoystickValue) / JoystickValue * DeadbandCutOff)) / (1 - DeadbandCutOff);
      }
      
          return deadbandreturn;
      }
  }

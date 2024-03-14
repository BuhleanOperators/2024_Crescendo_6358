// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;

import frc.robot.commands.Drive.arcadeDrive;
import frc.robot.commands.Intake.fullIntake;
import frc.robot.commands.Intake.runBelts;
import frc.robot.commands.Intake.runFlyWheels;
import frc.robot.commands.Pneumatics.FireSolenoid;
import frc.robot.commands.Pneumatics.RetractSolenoid;
import frc.robot.commands.Shooter.shooterRun;
import frc.robot.subsystems.pneumaticSubsystem;
import frc.robot.subsystems.beltSubsystem;
import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

  private double deadbandreturn;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static XboxController xDriver = new XboxController(OperatorConstants.kDriverControllerPort);
  private final static XboxController coPilot = new XboxController(OperatorConstants.kCoPilotControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    smartDashboard();
    
    Robot.m_DriveTrain.setDefaultCommand(new arcadeDrive(() -> deadband(getXDriver().getLeftY() * Constants.DriveConstants.maxSpeed, OperatorConstants.deadbandCutoffDrive), () -> deadband(getXDriver().getRightX() * Constants.DriveConstants.maxAngularSpeed, OperatorConstants.deadbandCutoffRot)));
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
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));
    
    JoystickButton intakeIn = new JoystickButton(xDriver, OperatorConstants.intakeIn);
    // JoystickButton intakeOut = new JoystickButton(xDriver, OperatorConstants.intakeOut);
    // intakeIn.onTrue(new runFlyWheels(0.75, m_IntakeSystem)).onFalse(new runFlyWheels(0, m_IntakeSystem));
    // intakeOut.onTrue(new runFlyWheels(-0.75)).onFalse(new runFlyWheels(0));
    intakeIn.onTrue(new fullIntake(0.75, 1)).onFalse(new fullIntake(0, 0));

    //new JoystickButton(xDriver, OperatorConstants.intakeButton).toggleOnTrue(new shooterRun(ShooterConstants.speed, m_ShooterSystem));
    new JoystickButton(xDriver, OperatorConstants.BUTTON_shooterSpeaker).onTrue(new shooterRun(ShooterConstants.speakerSpeed))
      .onFalse(new shooterRun(0));
    new JoystickButton(xDriver, OperatorConstants.BUTTON_shooterAmp).onTrue(new shooterRun(ShooterConstants.ampSpeed))
      .onFalse(new shooterRun(0));
    new JoystickButton(xDriver, OperatorConstants.BUTTON_shooterAmpSlow).onTrue(new shooterRun(ShooterConstants.ampSpeedSlow))
      .onFalse(new shooterRun(0));
    new JoystickButton(xDriver, OperatorConstants.BUTTON_beltsOut).onTrue(new runBelts(-1))
      .onFalse(new runBelts(0));

    
    new JoystickButton(coPilot, OperatorConstants.BUTTON_extendPiston).onTrue(new FireSolenoid());
    new JoystickButton(coPilot, OperatorConstants.BUTTON_retractPiston).onTrue(new RetractSolenoid());
    // new JoystickButton(xDriver, OperatorConstants.BUTTON_belts).onTrue(new runBelts(1, m_IntakeSystem))
      // .onFalse(new runBelts(0, m_IntakeSystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  // new JoystickButton(xDriver, OperatorConstants.BUTTON_togglePiston).onTrue(new ToggleSolenoid(m_pneumatics.getFirstSolenoid(), m_pneumatics));

  // new JoystickButton(coPilot, OperatorConstants.BUTTON_shooterStop).onTrue(new shooterRun(0, m_ShooterSystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    // return null;
    return Robot.m_SmartDashboard.getAutoCommand();
  }

  private void smartDashboard(){
    SmartDashboard.putNumber("Shooter RPM", Robot.m_ShooterSubsytem.getShooterRPM());
  }

  public static XboxController getXDriver(){
    return xDriver;
  }
  public static XboxController getCoPilot(){
    return coPilot;
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

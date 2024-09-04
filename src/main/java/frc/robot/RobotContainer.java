// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake.Belt;
import frc.robot.subsystems.Intake.BeltIO;
import frc.robot.subsystems.Intake.BeltIOVictorSP;
import frc.robot.subsystems.Intake.Flywheels;
import frc.robot.subsystems.Intake.FlywheelsIO;
import frc.robot.subsystems.Intake.FlywheelsIOVictorSP;
import frc.robot.subsystems.Pneumatics.Pneumatics;
import frc.robot.subsystems.Pneumatics.PneumaticsIO;
import frc.robot.subsystems.Pneumatics.PnueumaticsIOPnueumaticsHub;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIoSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSparkMax;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //^ Bind contoller inputs to commands
  public final Drive drive;
  public final Pneumatics pneumatics;
  public final Shooter shooter;
  public final Belt belt;
  public final Flywheels flywheel;

  // private double deadbandreturn;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static CommandXboxController xDriver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final static CommandXboxController coPilot = new CommandXboxController(OperatorConstants.kCoPilotControllerPort);

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode){
      case REAL:
        drive = new Drive(new DriveIOSparkMax());
        pneumatics = new Pneumatics(new PnueumaticsIOPnueumaticsHub());
        shooter = new Shooter(new ShooterIoSparkMax());
        belt = new Belt(new BeltIOVictorSP());
        flywheel = new Flywheels(new FlywheelsIOVictorSP());
        break;
      default : //REPLAY
        drive = new Drive(new DriveIO() {});
        pneumatics = new Pneumatics(new PneumaticsIO() {});
        shooter = new Shooter(new ShooterIO() {});
        belt = new Belt(new BeltIO() {});
        flywheel = new Flywheels(new FlywheelsIO() {});
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<Command>());

    autoChooser.addDefaultOption("Nothing", null);
    autoChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the trigger bindings
    configureBindings();
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

    drive.setDefaultCommand(
      Commands.run(
        () -> drive.arcadeDrive(xDriver.getLeftY(), xDriver.getRightX()), drive)
    );
    
    //Run shooter for amp
    xDriver
      .b()
      .onTrue(
        Commands.run(() -> shooter.runVelocity(1900), shooter))
      .onFalse(
        Commands.run(() -> shooter.runVelocity(0), shooter));

    //Run full intake (Belts and flywheels)
    xDriver
      .rightBumper()
      .onTrue(
        Commands.parallel(
          Commands.run(() -> belt.runSpeed(1.0), belt), 
          Commands.run(() -> flywheel.runSpeed(0.75), flywheel)))
      .onFalse(
        Commands.parallel(
          Commands.run(() -> belt.runSpeed(0.0), belt), 
          Commands.run(() -> flywheel.runSpeed(0.0), flywheel)));

    //Run belts back
    xDriver
      .leftBumper()
      .onTrue(
        Commands.run(() -> belt.runSpeed(-1.0), belt))
      .onFalse(
        Commands.run(() -> belt.runSpeed(0.0)));


    //Toggle climb state
    coPilot
      .a()
      .onTrue(
        Commands.runOnce(() -> pneumatics.toggleClimb(), pneumatics)
      );

    //! If method above doesn't work try this one
    // coPilot
    //   .a()
    //   .onTrue(
    //     Commands.either(
    //       Commands.run(() -> pneumatics.climbDown(), pneumatics), 
    //       Commands.run(() -> pneumatics.climbUp(), pneumatics), 
    //       () -> pneumatics.isClimbUp())
    //   );

    //Run shooter for speaker
    coPilot
      .b()
      .onTrue(
        Commands.run(() -> shooter.runVelocity(5100), shooter))
      .onFalse(
        Commands.run(() -> shooter.runVelocity(0), shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //Get the autonoumus command
    return autoChooser.get();
  }

  public static CommandXboxController getXDriver(){
    return xDriver;
  }
  public static CommandXboxController getCoPilot(){
    return coPilot;
  }

  // private double deadband(double JoystickValue, double DeadbandCutOff){
  //   //Create a deadband for the drive controls
  //   //If the joystick isn't pushed in either direction past a certain point gove the value 0, otherwise return the value of the joystick
  //   if (JoystickValue < DeadbandCutOff && JoystickValue > (DeadbandCutOff * (-1))) {
  //     deadbandreturn = 0;
  //     }
  //     else {
  //     deadbandreturn = (JoystickValue - (Math.abs(JoystickValue) / JoystickValue * DeadbandCutOff)) / (1 - DeadbandCutOff);
  //     }
      
  //         return deadbandreturn;
  //     }
  }

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Alert.AlertType;
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

import javax.print.attribute.standard.Copies;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final Alert driverDisconectedAlert = 
    new Alert("Driver controller disconected (Port 0)", AlertType.WARNING);
  private final Alert coPilotDisconectedAlert =
    new Alert("CoPilot controller disconected (Port 1)", AlertType.WARNING);
  private final LoggedDashboardNumber endgameAlert1 = 
    new LoggedDashboardNumber("Endagame alert #1", 30.0);
  private final LoggedDashboardNumber endgameAlert2 = 
    new LoggedDashboardNumber("Engame alert #2", 15.0);

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

    new Trigger(
      () -> 
        DriverStation.isTeleopEnabled()
        && DriverStation.getMatchTime() > 0
        && DriverStation.getMatchTime() <= (Math.round(endgameAlert1.get())))
      .onTrue(
        controllerRumbleCommand()
        .withTimeout(0.5)
      );
    new Trigger(
      () -> 
        DriverStation.isTeleopEnabled()
        && DriverStation.getMatchTime() > 0
        && DriverStation.getMatchTime() <= (Math.round(endgameAlert2.get())))
      .onTrue(
        controllerRumbleCommand()
        .withTimeout(0.5)
      );
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
      .whileTrue(
        Commands.startEnd(() -> shooter.runVelocity(1900), shooter::stop, shooter)
      );

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

    //Run shooter for speaker
    coPilot
      .b()
      .whileTrue(Commands.startEnd(() -> shooter.runVelocity(5100), shooter::stop, shooter));
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

  //**Updates dashboard data */
  public void updateDashboardOutputs(){
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("Up to Speaker Speed", shooter.getVelocityRPM() >= 5100);
  }

  //**Updates alerts for disconected controllers */
  public void checkControllers(){
    driverDisconectedAlert.set(
      !DriverStation.isJoystickConnected(xDriver.getHID().getPort())
        || !DriverStation.getJoystickIsXbox(xDriver.getHID().getPort()));
    coPilotDisconectedAlert.set(
      !DriverStation.isJoystickConnected(coPilot.getHID().getPort())
        || !DriverStation.getJoystickIsXbox(coPilot.getHID().getPort()));
  }

  private Command controllerRumbleCommand(){
    return Commands.startEnd(
      () -> {xDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
             coPilot.getHID().setRumble(RumbleType.kBothRumble, 1);
            }, 
      () -> {xDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
             coPilot.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            });
  }
}

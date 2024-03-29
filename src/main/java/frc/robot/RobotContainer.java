// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
//import frc.robot.Constants.ModuleConstant;
//import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SwerveJoystickCmd;
//import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

//import javax.swing.DefaultComboBoxModel;

//import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    //configureBindings();
    //defaultCommands();

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

      configureButtonBindings();
  }

  //private final SwerveModule swerveModule = new SwerveModule(0, 0, false, false, 0, 0, false);
 // private final SwerveJoystickCmd joystickCmd = new SwerveJoystickCmd(swerveSubsystem, null, null, null, null);
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */


private void configureButtonBindings() {
    //new CommandXboxController(2).leftStick(swerveSubsystem.zeroHeading());

    //new JoystickButton(driverJoystick, 2).onTrue(new SwerveJoystickCmd(swerveSubsystem, null, null, null, null));
    new JoystickButton(driverJoystick, 0).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
 
    //swerveSubsystem.zeroHeading(); coud go in instant as argument for InstantCommand (temporary fix?)
}

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Console;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.RotateRake;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Rake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //private final Drivetrain drivetrain;

  private final Controls controls;
  //private final Rake rake;
  private final Conveyor conveyor;

  private XboxController driver;
  private XboxController operator;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //drivetrain = new Drivetrain();
    //rake = new Rake();
    conveyor = new Conveyor();

    driver = new XboxController(0);
    operator = new XboxController(1);
    controls = new Controls(driver, operator);

    /*drivetrain.setDefaultCommand(new ArcadeDrive(
        drivetrain,
        controls::getDriveSpeed,
        controls::getTurnSpeed));*/

      //rake.setDefaultCommand(new RotateRake(rake, controls::getRakeRotationSpeed));

    configureButtonBindings(driver, operator);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(XboxController driver, XboxController operator) {
    // Example joystick button
    /*new JoystickButton(driver, Button.kA.value)
        .whenPressed(drivetrain::enable);*/

      new JoystickButton(driver, Button.kA.value).whenPressed(conveyor::Enable);
      new JoystickButton(driver, Button.kB.value).whenPressed(conveyor::Disable);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}

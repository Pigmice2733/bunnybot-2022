// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.RakeConfig;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.rake.RotateRakeAngle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Rake;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private final Drivetrain drivetrain;

  private final Controls controls;
  private final Drivetrain drivetrain;
  private final Rake rake;
  // private final ScoopIntake scoop;
  // private final Conveyor conveyor;

  private XboxController driver;
  private XboxController operator;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain = new Drivetrain();
    rake = new Rake();
    // scoop = new ScoopIntake();
    // conveyor = new Conveyor();

    driver = new XboxController(0);
    operator = new XboxController(1);
    controls = new Controls(driver, operator);

    drivetrain.setDefaultCommand(new ArcadeDrive(
        drivetrain,
        controls::getDriveSpeed,
        controls::getTurnSpeed));

    rake.setDefaultCommand(new RotateRakeAngle(rake::getAngle, rake));
    // scoop.setDefaultCommand(new RunScoop(scoop, () -> ScoopConfig.motorSpeed));

    configureButtonBindings(driver, operator);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(XboxController driver, XboxController operator) {
    /*
     * new JoystickButton(driver, Button.kA.value)
     * .whenPressed(drivetrain::enable);
     */

    /* new JoystickButton(driver, Button.kX.value)
        .whenPressed(new RunScoop(scoop, () -> 0));
    
    new JoystickButton(driver, Button.kRightBumper.value)
        .whenPressed(scoop::extend)
        .whenReleased(scoop::stopExtend);
    
    new JoystickButton(driver, Button.kLeftBumper.value)
        .whenPressed(scoop::retract)
        .whenReleased(scoop::stopExtend); */
    /*
     * new JoystickButton(driver, Button.kA.value)
     * .whenPressed(drivetrain::enable);
     */

    /* new JoystickButton(driver, Button.kA.value).whenPressed(conveyor::Enable);
    new JoystickButton(driver, Button.kB.value).whenPressed(conveyor::Disable);
    new JoystickButton(driver, Button.kY.value).whenPressed(conveyor::ToggleDirection); */

    new JoystickButton(operator, Button.kA.value)
        .whenPressed(new RotateRakeAngle(RakeConfig.intakeAngle, rake));
    new JoystickButton(operator, Button.kB.value)
        .whenPressed(new RotateRakeAngle(RakeConfig.raiseAngle, rake));
    new JoystickButton(operator, Button.kX.value)
        .whenPressed(new RotateRakeAngle(RakeConfig.startAngle, rake));
    new JoystickButton(operator, Button.kY.value)
        .whenPressed(new RotateRakeAngle(RakeConfig.dispenseAngle, rake));

    new JoystickButton(operator, Button.kRightBumper.value)
        .whenPressed(new InstantCommand(rake::toggleMode));
    new JoystickButton(operator, Button.kLeftBumper.value)
        .whenPressed(new InstantCommand(rake::toggleDisabled));
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

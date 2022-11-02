// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.RakeConfig;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.TurnDegrees;
import frc.robot.commands.drivetrain.AutoRoutines.TestPath;
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
  private final Controls controls;
  private final Drivetrain drivetrain;
  private final Rake rake;

  private XboxController driver;
  private XboxController operator;

  public RobotContainer() {
    drivetrain = new Drivetrain();
    rake = new Rake();

    driver = new XboxController(0);
    operator = new XboxController(1);
    controls = new Controls(driver, operator);

    drivetrain.setDefaultCommand(new ArcadeDrive(
      drivetrain,
      controls::getDriveSpeed,
      controls::getTurnSpeed));

    rake.setDefaultCommand(new RotateRakeAngle(rake::getAngle, rake));

    configureButtonBindings(driver, operator);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(XboxController driver, XboxController operator) {

    // OPERATOR CONTROLS

    // Rotate rake to preset positions
    new JoystickButton(operator, Button.kA.value)
      .whenPressed(new RotateRakeAngle(RakeConfig.intakeAngle, rake));
    new JoystickButton(operator, Button.kB.value)
      .whenPressed(new RotateRakeAngle(RakeConfig.raiseAngle, rake));
    new JoystickButton(operator, Button.kX.value)
      .whenPressed(new RotateRakeAngle(RakeConfig.startAngle, rake));
    new JoystickButton(operator, Button.kY.value)
      .whenPressed(new RotateRakeAngle(RakeConfig.dispenseAngle, rake));

    // Toggle rake mode
    new JoystickButton(operator, Button.kRightBumper.value)
      .whenPressed(new InstantCommand(rake::toggleMode));
    new JoystickButton(operator, Button.kLeftBumper.value)
      .whenPressed(new InstantCommand(rake::toggleDisabled));

    // DRIVER CONTROLS

    // Rotate 90 degrees with bumper press
    new JoystickButton(driver, Button.kRightBumper.value)
      .whenPressed(() -> CommandScheduler.getInstance().schedule(new TurnDegrees(drivetrain, 90).withTimeout(1)));
    new JoystickButton(driver, Button.kLeftBumper.value)
      .whenPressed(() -> CommandScheduler.getInstance().schedule(new TurnDegrees(drivetrain, -90).withTimeout(1)));
  
    // Slow mode
    new JoystickButton(driver, Button.kY.value)
      .whenPressed(new InstantCommand(drivetrain::enableSlow))
      .whenReleased(new InstantCommand(drivetrain::disableSlow));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new TestPath(drivetrain);
  }
}

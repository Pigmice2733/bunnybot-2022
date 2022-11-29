// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.RakeConfig;
import frc.robot.commands.AutoRoutines.AutoDispense;
import frc.robot.commands.AutoRoutines.DriveAndDispense;
import frc.robot.commands.AutoRoutines.TestPath;
import frc.robot.commands.HardStopper.RetractHardStop;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.drivetrain.TurnDegrees;
import frc.robot.commands.rake.RotateBackwardsLimitSwitch;
import frc.robot.commands.rake.RotateForwardLimitSwitch;
import frc.robot.commands.rake.RotateRakeManual;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HardStop;
import frc.robot.subsystems.LEDlights;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
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
  private final HardStop hardStop;

  private XboxController driver;
  private XboxController operator;

  private AutoDispense autoDispense;

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    drivetrain = new Drivetrain();
    rake = new Rake();
    hardStop = new HardStop();

    driver = new XboxController(0);
    operator = new XboxController(1);

    controls = new Controls(driver, operator);

    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, controls::getDriveSpeed, controls::getTurnSpeed));
    rake.setDefaultCommand(new RotateRakeManual(controls::getRakeRotationSpeed, rake));

    List<Command> autoCommands = List.of(
      new DriveAndDispense(drivetrain, rake, hardStop),
      new DriveDistance(drivetrain, 3)
    );

    autoCommands.forEach(command -> {
			autoChooser.addOption(command.getName(), command);
		});

    autoChooser = new SendableChooser<Command>();
    autoChooser.addOption("None", new WaitCommand(1));
    
    autoDispense = new AutoDispense(drivetrain, rake);

    configureButtonBindings(driver, operator);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings(XboxController driver, XboxController operator) {
    // Rake preset angles (automatically switches )
    new JoystickButton(operator, Button.kA.value)
        .whenPressed(new RotateBackwardsLimitSwitch(rake));
    new JoystickButton(operator, Button.kB.value)
        .whenPressed(new RotateForwardLimitSwitch(rake));
    new JoystickButton(operator, Button.kY.value)
        .whenPressed(() -> rake.setSetpoint(RakeConfig.startAngle));
    new JoystickButton(operator, Button.kX.value)
        .whenPressed(() -> rake.setSetpoint(RakeConfig.dispenseAngle));

    // Emergency release hardStop pistons
    new JoystickButton(operator, Button.kStart.value)
        .whenPressed(new RetractHardStop(hardStop));

    new JoystickButton(operator, Button.kA.value)
      .whenPressed(new RotateForwardLimitSwitch(rake));

    new JoystickButton(operator, Button.kX.value)
      .whenPressed(new RotateBackwardsLimitSwitch(rake));

    // Auto turn 90 degrees
    new JoystickButton(driver, Button.kRightBumper.value)
      .whenPressed(() -> CommandScheduler.getInstance().schedule(new TurnDegrees(drivetrain, 180).withTimeout(1)));
    
    new JoystickButton(driver, Button.kLeftBumper.value)
      .whenPressed(() -> CommandScheduler.getInstance().schedule(new TurnDegrees(drivetrain, -180).withTimeout(1))); 

    // Auto dispense (hold, release to cancel)
    new JoystickButton(operator, Button.kBack.value)
      .whenPressed(() -> CommandScheduler.getInstance().schedule(autoDispense))
      .whenReleased(() -> CommandScheduler.getInstance().cancel(autoDispense));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
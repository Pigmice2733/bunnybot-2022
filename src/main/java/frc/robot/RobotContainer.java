// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.RakeConfig;
import frc.robot.commands.AutoRoutines.AutoDispense;
import frc.robot.commands.AutoRoutines.DriveAndDispense;
import frc.robot.commands.AutoRoutines.TestPath;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.TurnDegrees;
import frc.robot.commands.rake.RotateRakeAngle;
import frc.robot.commands.rake.RotateRakeManual;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HardStop;
import frc.robot.subsystems.RakeOld;
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
  // private final Rake rake;
  //HardStop hardstop = new HardStop();


  private final AutonomousChooser chooser;

  private XboxController driver;
  private XboxController operator;

  //private AutoDispense autoDispense = new AutoDispense(drivetrain);

  public RobotContainer() {
    drivetrain = new Drivetrain();
    //rake = new Rake();

    chooser = new AutonomousChooser();

    driver = new XboxController(0);
    operator = new XboxController(1);
    controls = new Controls(driver, operator);

    /* drivetrain.setDefaultCommand(new ArcadeDrive(
        drivetrain,
        controls::getDriveSpeed,
        controls::getTurnSpeed)); */

    // rake.setDefaultCommand(new RotateRakeManual(controls::getRakeRotationSpeed, rake));

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
    // new JoystickButton(operator, Button.kA.value)
    //     .whenPressed(() -> rake.setLimitSwitchModeDown());
    // new JoystickButton(operator, Button.kB.value)
    //     .whenPressed(() -> rake.setLimitSwitchModeUp());
    // new JoystickButton(operator, Button.kY.value)
    //     .whenPressed(() -> rake.setSetpoint(RakeConfig.startAngle));
    // new JoystickButton(operator, Button.kX.value)
    //     .whenPressed(() -> rake.setSetpoint(RakeConfig.dispenseAngle));

    //Emergency Release the Hard stop pistons
    // new JoystickButton(operator, Button.kStart.value)
    //     .whenPressed(hardstop::retractStop);


    // Toggle rake mode
    // new JoystickButton(operator, Button.kRightBumper.value)
    //     .whenPressed(rake::toggleMode);
    // new JoystickButton(operator, Button.kLeftBumper.value)
    //     .whenPressed(rake::toggleDisabled);

    // Auto turn 90 degrees
    //  new JoystickButton(driver, Button.kRightBumper.value)
    //     .whenPressed() -> CommandScheduler.getInstance().schedule(new TurnDegrees(drivetrain, 180).withTimeout(1)));
    // new JoystickButton(driver, Button.kLeftBumper.value)
    //     .whenPressed(() -> CommandScheduler.getInstance().schedule(new TurnDegrees(drivetrain, -180).withTimeout(1))); 

    // Auto dispense (hold, release to cancel)
    // new JoystickButton(operator, Button.kBack.value)
    //   .whenPressed(() -> CommandScheduler.getInstance().schedule(autoDispense))
    //   .whenReleased(() -> CommandScheduler.getInstance().cancel(autoDispense));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /* drivetrain.resetOdometry();
    
    TrajectoryConfig config = new TrajectoryConfig(1.7, 0.7);
    config.setKinematics(drivetrain.getKinematics());
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(), new Pose2d(2, -2, new Rotation2d(-45)), new Pose2d(2.5, 0, new Rotation2d(90))),
        config);
    
    return new FollowPath(drivetrain, trajectory); */
  
    //hardstop.retractStop();
    return chooser.chooser.getSelected();
  }
}

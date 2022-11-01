// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.RakeConfig;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.DriveDistanceNew;
import frc.robot.commands.drivetrain.TurnDegrees;
import frc.robot.commands.drivetrain.DriveDistance;
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
  //private final Rake rake;
  // private final ScoopIntake scoop;
  // private final Conveyor conveyor;

  private XboxController driver;
  private XboxController operator;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain = new Drivetrain();
    //rake = new Rake();
    // scoop = new ScoopIntake();
    // conveyor = new Conveyor();

    driver = new XboxController(0);
    operator = new XboxController(1);
    controls = new Controls(driver, operator);

    drivetrain.setDefaultCommand(new ArcadeDrive(
        drivetrain,
        controls::getDriveSpeed,
        controls::getTurnSpeed));

    //rake.setDefaultCommand(new RotateRakeAngle(rake::getAngle, rake));
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

    // new JoystickButton(operator, Button.kA.value)
    //     .whenPressed(new RotateRakeAngle(RakeConfig.intakeAngle, rake));
    // new JoystickButton(operator, Button.kB.value)
    //     .whenPressed(new RotateRakeAngle(RakeConfig.raiseAngle, rake));
    // new JoystickButton(operator, Button.kX.value)
    //     .whenPressed(new RotateRakeAngle(RakeConfig.startAngle, rake));
    // new JoystickButton(operator, Button.kY.value)
    //     .whenPressed(new RotateRakeAngle(RakeConfig.dispenseAngle, rake));

    // new JoystickButton(operator, Button.kRightBumper.value)
    //     .whenPressed(new InstantCommand(rake::toggleMode));
    // new JoystickButton(operator, Button.kLeftBumper.value)
    //     .whenPressed(new InstantCommand(rake::toggleDisabled));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new DriveDistance(drivetrain, 10);
    // drivetrain.resetOdometry();

    // TrajectoryConfig config = new TrajectoryConfig(0.5, 2);
    //     config.setKinematics(drivetrain.getKinematics());

    //     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //         List.of(new Pose2d(), new Pose2d(5, 0, new Rotation2d(Math.PI/2))),
    //         config
    //     );
    //     System.out.println(trajectory.toString());


    //     RamseteCommand command = new RamseteCommand(
    //         trajectory,
    //         drivetrain::getPose,
    //         new RamseteController(DrivetrainConfig.kB, DrivetrainConfig.kZeta),
    //         drivetrain.getFeedForward(),
    //         drivetrain.getKinematics(),
    //         drivetrain::getWheelSpeeds,
    //         new PIDController(DrivetrainConfig.kP, DrivetrainConfig.kI, DrivetrainConfig.kD), // Left
    //         new PIDController(DrivetrainConfig.kP, DrivetrainConfig.kI, DrivetrainConfig.kD), // Right
    //         drivetrain::tankDriveVolts,
    //         drivetrain
    //     );

    //     return command;
        drivetrain.resetOdometry();
        

        //return new DriveDistanceOld(drivetrain, 2);
        //return new DriveDistance(drivetrain, 2);
        return new TurnDegrees(drivetrain, 90);
  }
}

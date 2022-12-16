// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RakeConfig;
import frc.robot.commands.auto_routines.AutoDispense;
import frc.robot.commands.auto_routines.DriveAndDispense;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.drivetrain.TurnDegrees;
import frc.robot.commands.hard_stop.ExtendHardStop;
import frc.robot.commands.hard_stop.RetractHardStop;
import frc.robot.commands.rake.RotateBackwardsLimitSwitch;
import frc.robot.commands.rake.RotateForwardLimitSwitch;
import frc.robot.commands.rake.RotateRakeManual;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HardStop;
import frc.robot.subsystems.Rake;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.RakeConfig;
import frc.robot.commands.auto_routines.AutoDispense;
import frc.robot.commands.auto_routines.DriveAndDispense;
import frc.robot.commands.hard_stop.RetractHardStop;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.drivetrain.TurnDegrees;
import frc.robot.commands.rake.RaiseRakeSlightly;
import frc.robot.commands.rake.RotateBackwardsLimitSwitch;
import frc.robot.commands.rake.RotateForwardLimitSwitch;
import frc.robot.commands.rake.RotateRakeManual;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HardStop;
import frc.robot.subsystems.Rake;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static ProfiledPIDController driveDistController = new ProfiledPIDController(DrivetrainConfig.driveDistP, DrivetrainConfig.driveDistI, DrivetrainConfig.driveDistD, new Constraints(1, 1));
  //public static ProfiledPIDController turnDegreesController = new ProfiledPIDController(DrivetrainConfig.turnP, DrivetrainConfig.turnI, DrivetrainConfig.turnD, new Constraints(15, 2));
  public static PIDController turnDegreesController = new PIDController(DrivetrainConfig.turnP, DrivetrainConfig.turnI, DrivetrainConfig.turnD);
  
  private final Controls controls;
  private final Drivetrain drivetrain;
  private final Rake rake;
  private final HardStop hardStop;

  private XboxController driver;
  private XboxController operator;

  // private AutoDispense autoDispense;

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //Shuffleboard.getTab("Drivetrain").add("Drive Distance", driveDistController);
    Shuffleboard.getTab("Drivetrain").add("Turn Degrees", turnDegreesController);

    drivetrain = new Drivetrain();
    rake = new Rake();
    hardStop = new HardStop();

    driver = new XboxController(0);
    operator = new XboxController(1);

    controls = new Controls(driver, operator);

    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, controls::getDriveSpeed, controls::getTurnSpeed));
    rake.setDefaultCommand(new RotateRakeManual(controls::getRakeRotationSpeed, rake));

    List<Command> autoCommands = List.of(
      //new DriveAndDispense(drivetrain, rake, hardStop),
      //new DriveDistance(drivetrain, Units.inchesToMeters(258))
      new DriveDistance(drivetrain, 2),
      new TurnDegrees(drivetrain, 90)
      //new RetractHardStop(hardStop)
    );

    autoChooser = new SendableChooser<Command>();
    Shuffleboard.getTab("Drivetrain").add("Auto Chooser", autoChooser);

    autoCommands.forEach(command -> {
      System.out.println(command.getName());
      autoChooser.addOption(command.getName(), command);
    });

    autoChooser.addOption("None", new WaitCommand(1));

    // autoDispense = new AutoDispense(drivetrain, rake);

    configureButtonBindings(driver, operator);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings(XboxController driver, XboxController operator) {
    // Rotate rake (automatically switches to auto or limit switch mode)
    new JoystickButton(operator, Button.kA.value)
        .whenPressed(new RotateBackwardsLimitSwitch(rake));
    new JoystickButton(operator, Button.kY.value)
        .whenPressed(new RotateForwardLimitSwitch(rake));
    
    new JoystickButton(operator, Button.kB.value)
      .whenPressed(new RaiseRakeSlightly(rake));

    new JoystickButton(operator, Button.kX.value)
      .whenPressed(new RaiseRakeSlightly(rake))
      .whenReleased(new RotateBackwardsLimitSwitch(rake));

    // Emergency release hardStop pistons
    // new JoystickButton(operator, Button.kStart.value)
    //     .whenPressed(new RetractHardStop(hardStop));

    // // Auto turn 90 degrees right
    // new JoystickButton(driver, Button.kRightBumper.value)
    //   .whenPressed(() -> CommandScheduler.getInstance().schedule(new TurnDegrees(drivetrain, 270).withTimeout(8)));
    
    // // Auto turn 90 degrees right
    // new JoystickButton(driver, Button.kLeftBumper.value)
    //   .whenPressed(() -> CommandScheduler.getInstance().schedule(new TurnDegrees(drivetrain, 90).withTimeout(8))); 


    // // Slow mode (1/4 speed)
    // new JoystickButton(driver, Button.kY.value)
    //   .whenPressed(() -> drivetrain.enableSlow())
    //   .whenReleased(() -> drivetrain.disableSlow());

    // // Auto dispense (hold, release to cancel)
    // new JoystickButton(operator, Button.kBack.value)
    //   .whenPressed(() -> CommandScheduler.getInstance().schedule(autoDispense))
    //   .whenReleased(() -> CommandScheduler.getInstance().cancel(autoDispense));

    new JoystickButton(driver, Button.kA.value).whenPressed(new RetractHardStop(hardStop));
    new JoystickButton(driver, Button.kX.value).whenPressed(new ExtendHardStop(hardStop));

    new JoystickButton(driver, Button.kX.value)
      .whenPressed(drivetrain::resetOdometry);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new SequentialCommandGroup(new RetractHardStop(hardStop),
    // autoChooser.getSelected());
    return autoChooser.getSelected();
    // return new DriveDistance(drivetrain, 3);
    // return new RetractHardStop(hardStop);
  }
}

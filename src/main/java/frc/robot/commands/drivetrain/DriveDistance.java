// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends ProfiledPIDCommand {
  private Drivetrain drivetrain;

  /**
   * Use PIDControllers to drive the specified distance.
   * @param drivetrain a drivetrain subsystem
   * @param distance The distance to drive. The robot will move forward if this is positive or backward if this is negative.
   */
  public DriveDistance(Drivetrain drivetrain, double distance) {
    super(
      new ProfiledPIDController(DrivetrainConfig.driveDistP, DrivetrainConfig.driveDistI, DrivetrainConfig.driveDistD, new Constraints(1, 1.5)), 
        drivetrain::getAverageDistance,
        distance, 
        (output,setpoint) -> { drivetrain.arcadeDrive(output, 0); },
        drivetrain
    );

    //Shuffleboard.getTab("Drivetrain").add("Drive Distance PID", getController());

    getController().setTolerance(0.05, 0.1);

    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    //getController().setSetpoint((distance + drivetrain.getAverageDistance()));
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

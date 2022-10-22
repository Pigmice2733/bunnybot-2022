// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends ProfiledPIDCommand {
  private final double errorThreshold = 0.05;
  private final double turnSpeedThreshold = 1.0;

  public TurnToAngle(double targetHeading, boolean absolute, Drivetrain drivetrain) {
    super(
        new ProfiledPIDController(
            0.1,
            0.02,
            0.0,
            new TrapezoidProfile.Constraints(1.0, 1.5)),
        drivetrain::getHeading,
        absolute ? targetHeading : targetHeading + drivetrain.getHeading(),
        (output, setpoint) -> drivetrain.arcadeDrive(0, output),
        drivetrain);

    getController().enableContinuousInput(-180, 180);

    getController().setTolerance(errorThreshold, turnSpeedThreshold);
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
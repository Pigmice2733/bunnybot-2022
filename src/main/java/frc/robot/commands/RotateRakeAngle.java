// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Rake;

public class RotateRakeAngle extends PIDCommand {
  private final double errorThreshold = 0.05;
  private final double turnSpeedThreshold = 1.0;
  private final Rake rake;

  public RotateRakeAngle(double targetAngle, Rake rake) {
    super(
        new PIDController(0.1, 0.02, 0.0),
        rake::getAngle,
        targetAngle,
        (output) -> rake.setMotorSpeed(output),
        rake);

    getController().setTolerance(errorThreshold, turnSpeedThreshold);

    this.rake = rake;
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.rake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.Rake;

public class RotateRakeAngle extends PIDCommand {
  private final double errorThreshold = 0.05;
  private final double turnSpeedThreshold = 1.0;

  public RotateRakeAngle(double targetAngle, Rake rake) {
    super(
        new PIDController(0.1, 0.02, 0.0),
        rake::getAngle,
        targetAngle,
        (output) -> rake.setSpeedAuto(output),
        rake);

    getController().setTolerance(errorThreshold, turnSpeedThreshold);
  }

  public RotateRakeAngle(DoubleSupplier targetAngle, Rake rake) {
    this(targetAngle.getAsDouble(), rake);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
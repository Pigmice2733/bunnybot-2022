// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends PIDCommand {
  private Drivetrain drivetrain;

  public DriveDistance(Drivetrain drivetrain, double distance) {
    super(
      new PIDController(0.3, 0.02, 0), 
          drivetrain::getAverageDistance,
          distance, 
          (output) -> { drivetrain.arcadeDrive(output, 0); },
          drivetrain
    );
    this.m_controller.setTolerance(0.05, 0.1);
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    drivetrain.resetOdometry();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return this.m_controller.atSetpoint();
  }
}

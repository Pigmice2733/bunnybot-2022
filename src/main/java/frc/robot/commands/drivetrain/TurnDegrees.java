// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class TurnDegrees extends PIDCommand {
  private Drivetrain drivetrain;

  public TurnDegrees(Drivetrain drivetrain, double rotation) {
    super(
      new PIDController(0.006, 0, 0.0001), 
          drivetrain::getHeadingDegrees,
          (rotation + drivetrain.getHeadingDegrees()) % 360, 
          (output) -> { drivetrain.arcadeDrive(0, output); },
          drivetrain
    );
    this.m_controller.setTolerance(3, 0.1);
    this.m_controller.enableContinuousInput(0, 360);
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.m_controller.atSetpoint();
  }
}

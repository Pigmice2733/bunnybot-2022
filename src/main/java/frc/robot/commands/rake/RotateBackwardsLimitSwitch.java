// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.rake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RakeConfig;
import frc.robot.Constants.RakeConfig.RakeMode;
import frc.robot.subsystems.Rake;

public class RotateBackwardsLimitSwitch extends CommandBase {
private final Rake rake;
  /**
   * Rotate the rake down until the bottom limit switches are pressed.
   * @param rake a rake subsystem
   */
  public RotateBackwardsLimitSwitch(Rake rake) {
    this.rake = rake;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rake.setMode(RakeMode.limitSwitch);
    rake.setOutputs(-RakeConfig.autoRotateSpeed, -RakeConfig.autoRotateSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return ((rake.getBottomLeftSwitch() || rake.getBottomRightSwitch()) || rake.getMode() != RakeMode.limitSwitch);
  }
}

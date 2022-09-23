// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Rake;

public class RotateRake extends CommandBase {
  public Rake rake;
  private DoubleSupplier speed;

  public RotateRake(Rake rake, DoubleSupplier speed) {
    this.rake = rake;
    this.speed = speed;

    addRequirements(rake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rake.SetMotorSpeed(speed.getAsDouble());
  }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {}
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return false;
   }
}

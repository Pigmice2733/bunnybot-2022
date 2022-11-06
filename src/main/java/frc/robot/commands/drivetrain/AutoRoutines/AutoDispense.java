// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.drivetrain.TurnDegrees;
import frc.robot.subsystems.Drivetrain;

public class AutoDispense extends SequentialCommandGroup {
  /** Creates a new AutoDispense. */
  public AutoDispense(Drivetrain drivetrain) {
    addCommands(
      new DriveDistance(drivetrain, -0.5).withTimeout(1),
      new TurnDegrees(drivetrain, 180),
      new DriveDistance(drivetrain, 1).withTimeout(1)
      //new RotateRakeAngle(RakeConfig.dispenseAngle, rake)
      );
  }
}

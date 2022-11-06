// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.AutoRoutines;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.RakeConfig;
import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.rake.RotateRakeAngle;
import frc.robot.subsystems.Drivetrain;

public class DriveAndDispense extends SequentialCommandGroup {
  public DriveAndDispense(Drivetrain drivetrain) {
    addCommands(
      new DriveDistance(drivetrain, 5)
      //new RotateRakeAngle(RakeConfig.dispenseAngle, rake)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RakeConfig;
import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Rake;

public class DriveAndDispense extends SequentialCommandGroup {
  public DriveAndDispense(Drivetrain drivetrain, Rake rake) {
    addCommands(
      new DriveDistance(drivetrain, 5),
      new InstantCommand(() -> rake.setSetpoint(RakeConfig.dispenseAngle))
    );
  }
}

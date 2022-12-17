// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.rake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RakeConfig;
import frc.robot.subsystems.Rake;

public class RaiseRakeSlightly extends SequentialCommandGroup {
  public RaiseRakeSlightly(Rake rake) {
    addCommands(new InstantCommand(() -> rake.setOutputs(RakeConfig.autoRotateSpeed, RakeConfig.autoRotateSpeed),rake),
    new WaitCommand(1),
    new InstantCommand(() -> rake.setOutputs(0, 0))
    );
  }
}

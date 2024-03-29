// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_routines;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.rake.RotateForwardLimitSwitch;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HardStop;
import frc.robot.subsystems.Rake;

public class DriveAndDispense extends SequentialCommandGroup {
  /**
   * Autonomous routine. Drive backward 5 meters and lower the rake to dispense a tube.
   * @param drivetrain a drivetrain subsystem
   * @param rake a rake subsystem
   * @param hardStop a hard-stop subsystem
   */
  public DriveAndDispense(Drivetrain drivetrain, Rake rake) {
    addCommands(
      new DriveDistance(drivetrain, Units.inchesToMeters(-258)),
      new RotateForwardLimitSwitch(rake)
    );
  }
}

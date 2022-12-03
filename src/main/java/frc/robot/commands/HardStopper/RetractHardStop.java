// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.HardStopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HardStop;

public class RetractHardStop extends CommandBase {
  private HardStop hardStop;

  /**
   * Retract the hard-stop pistons.
   * @param hardStop a hard-stop subsystem
   */
  public RetractHardStop(HardStop hardStop) {
    this.hardStop = hardStop;
    addRequirements(hardStop);  
  }

  @Override
  public void initialize() {
    hardStop.retractStop(); //Retracts Pistons (HardStop deactivated)
  }
  
  @Override
  public boolean isFinished() {
    return true;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardStopConfig;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;


import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class HardStop extends SubsystemBase {

  private final DoubleSolenoid frontSolenoid, backSolenoid;

  /** Creates a new HardStop. */
  public HardStop() {
    frontSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, HardStopConfig.solenoidPortFrontLeft, HardStopConfig.solenoidPortFrontRight);
    backSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, HardStopConfig.solenoidPortBackLeft, HardStopConfig.solenoidPortBackRight);
  }

  public void setStop(){
    frontSolenoid.set(kForward);
    backSolenoid.set(kForward);
  }

  public void release(){
    frontSolenoid.set(kReverse);
    backSolenoid.set(kReverse);
  }
}

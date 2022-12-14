// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardStopConfig;
import frc.robot.Constants.ShuffleboardConfig;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class HardStop extends SubsystemBase {
  private final DoubleSolenoid frontSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, HardStopConfig.solenoidPortFrontLeft, HardStopConfig.solenoidPortFrontRight);
  private final DoubleSolenoid backSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, HardStopConfig.solenoidPortBackLeft, HardStopConfig.solenoidPortBackRight);
  private final Compressor compressor;

  //private final NetworkTableEntry pressureEntry;

  public HardStop() { 
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  }

  public void periodic() {
   
  }

  public void engageStop(){
    frontSolenoid.set(Value.kForward);
    backSolenoid.set(Value.kForward);
  }

  public void retractStop(){
    frontSolenoid.set(Value.kReverse);
    backSolenoid.set(Value.kReverse);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardStopConfig;
import frc.robot.Constants.ShuffleboardConfig;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import java.time.Period;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;

public class HardStop extends SubsystemBase {

  private final DoubleSolenoid frontSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, HardStopConfig.solenoidPortFrontLeft, HardStopConfig.solenoidPortFrontRight);
  private final DoubleSolenoid backSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, HardStopConfig.solenoidPortBackLeft, HardStopConfig.solenoidPortBackRight);
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  private final NetworkTableEntry pressureEntry;

  /** Creates a new HardStop. */
  public HardStop() { 
    pressureEntry = Shuffleboard.getTab("Pneumatics").add("Pressure", -1.0).getEntry();
  }

  public void periodic() {
    if (ShuffleboardConfig.pneumaticsPrintsEnabled)
      Shuffleboard.getTab("Pneumatics").add("Pressure", compressor.getPressure());
  }

  public void engageStop(){
    frontSolenoid.set(kForward);
    backSolenoid.set(kForward);
  }

  public void retractStop(){
    frontSolenoid.set(kReverse);
    backSolenoid.set(kReverse);
  }
}

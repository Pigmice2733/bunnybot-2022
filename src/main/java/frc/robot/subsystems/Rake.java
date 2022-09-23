// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RakeConfig;

public class Rake extends SubsystemBase {
  private final TalonSRX motor;

  /** Creates a new Rake. */
  public Rake() {
    motor = new TalonSRX(RakeConfig.motorID);
  }

  @Override
  public void periodic() {

  }

  public void SetMotorSpeed(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }
}

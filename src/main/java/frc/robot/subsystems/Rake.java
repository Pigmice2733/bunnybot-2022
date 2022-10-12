// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RakeConfig;

public class Rake extends SubsystemBase {
  private final TalonSRX leftMotor;
  private final TalonSRX rightMotor;
  private double angle;

  /** Creates a new Rake. */
  public Rake() {
    leftMotor = new TalonSRX(RakeConfig.leftMotorID);
    rightMotor = new TalonSRX(RakeConfig.rightMotorID);

    angle = RakeConfig.startAngle;
  }

  @Override
  public void periodic() {
    this.angle = (leftMotor.getSelectedSensorPosition() + rightMotor.getSelectedSensorPosition())
        * 45 / 1024; // average angle for two motors (should be the same anyway)
  }

  public void setMotorSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getAngle() {
    return this.angle;
  }
}

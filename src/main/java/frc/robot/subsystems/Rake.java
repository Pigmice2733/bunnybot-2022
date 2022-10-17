// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RakeConfig;
import frc.robot.Constants.RakeConfig.RakeMode;

public class Rake extends SubsystemBase {
  private final TalonSRX leftMotor;
  private final TalonSRX rightMotor;
  private double angle;

  RakeMode mode = RakeMode.Manual;

  private final ShuffleboardTab rakeTab;
  private final NetworkTableEntry angleEntry;

  /** Creates a new Rake. */
  public Rake() {
    leftMotor = new TalonSRX(RakeConfig.leftMotorID);
    rightMotor = new TalonSRX(RakeConfig.rightMotorID);

    angle = RakeConfig.startAngle;

    rakeTab = Shuffleboard.getTab("Rake");
    angleEntry = rakeTab.add("Angle", angle).getEntry();
  }

  @Override
  public void periodic() {
    updateAngle();

    if (mode == RakeMode.Disabled)
      setMotorSpeeds(0);
  }

  public void setSpeedManual(double speed) {
    if (mode != RakeMode.Manual)
      return;
    
      setMotorSpeeds(speed);
  }

  public void setSpeedAuto(double speed) {
    if (mode != RakeMode.Automatic)
      return;

      setMotorSpeeds(speed);
  }

  void setMotorSpeeds(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void updateAngle() {
    this.angle = (leftMotor.getSelectedSensorPosition() + rightMotor.getSelectedSensorPosition())
        * 45 / 1024; // average angle for two motors (should be the same anyway)
        
    angleEntry.setDouble(angle);
  }

  public double getAngle() {
    return this.angle;
  }

  public void setMode(RakeMode mode) {
    this.mode = mode;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.RakeConfig;
import frc.robot.Constants.ShuffleboardConfig;
import frc.robot.Constants.RakeConfig.RakeMode;

public class RakeOld extends SubsystemBase {
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  private final ShuffleboardTab rakeTab;
  private final NetworkTableEntry angleEntry;
  private final NetworkTableEntry modeEntry;

  private RakeMode mode;
  private boolean disabled;
  private double angle;

  /** Creates a new Rake. */
  public RakeOld() {
    leftMotor = new CANSparkMax(RakeConfig.leftMotorID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(RakeConfig.rightMotorID, MotorType.kBrushless);

    // sets encoders to report in degrees
    leftMotor.getEncoder().setPositionConversionFactor(360);
    rightMotor.getEncoder().setPositionConversionFactor(360);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    angle = RakeConfig.startAngle;
    mode = RakeMode.Manual;

    rakeTab = Shuffleboard.getTab("Rake");
    angleEntry = rakeTab.add("Angle", angle).getEntry();
    modeEntry = rakeTab.add("Mode", "manual").getEntry();
  }

  @Override
  public void periodic() {
    updateAngle();

    if (disabled)
      setMotorSpeeds(0);
  }

  public void setSpeedManual(double speed) {
    if (mode == RakeMode.Manual)
      setMotorSpeeds(speed);
  }

  public void setSpeedAuto(double speed) {
    if (mode == RakeMode.Automatic)
      setMotorSpeeds(speed);
  }

  void setMotorSpeeds(double speed) {
    leftMotor.set(speed);
    // rightMotor.set(speed);
  }

  public void updateAngle() {
    // average angle for two motors (should be the same anyway)
    this.angle = (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getPosition()) / 2;

    if (ShuffleboardConfig.rakePrintsEnabled)
      angleEntry.setDouble(angle);
  }

  public double getAngle() {
    return this.angle;
  }

  public void setMode(RakeMode mode) {
    this.mode = mode;

    if (ShuffleboardConfig.rakePrintsEnabled) {
      if (this.disabled) {
        modeEntry.setString("disabled");
      } else {
        modeEntry.setString(mode == RakeMode.Automatic ? "automatic" : "manual");
      }
    }
  }

  public void toggleMode() {
    if (this.mode == RakeMode.Automatic) {
      setMode(RakeMode.Manual);
    }
    if (this.mode == RakeMode.Manual) {
      setMode(RakeMode.Automatic);
    }
  }

  public void toggleDisabled() {
    this.disabled = !this.disabled;

    modeEntry.setString("disabled");
  }
}

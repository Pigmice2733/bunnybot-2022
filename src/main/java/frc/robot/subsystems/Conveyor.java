// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConfig;

public class Conveyor extends SubsystemBase {
  private final TalonSRX bottomMotor, topMotor;

  boolean enabled;
  boolean isBackwards;

  public void Enable() {
    SetMotorSpeeds(ConveyorConfig.speed);
    enabled = true;
  }

  public void Disable() {
    SetMotorSpeeds(0.0);
    enabled = false;
  }

  public void ToggleDirection() {
    isBackwards = !isBackwards;

    if (enabled) {
      SetMotorSpeeds(ConveyorConfig.speed);
    }
  }

  /** Creates a new Conveyer. */
  public Conveyor() {
    bottomMotor = new TalonSRX(ConveyorConfig.botMotorID);
    topMotor = new TalonSRX(ConveyorConfig.topMotorID);

    bottomMotor.setInverted(ConveyorConfig.botMotorInverted);
    topMotor.setInverted(ConveyorConfig.topMotorInverted);
  }

  public void SetMotorSpeeds(Double speed) {
    if (isBackwards) {
      speed = -speed;
    }

    bottomMotor.set(ControlMode.PercentOutput, speed);
    topMotor.set(ControlMode.PercentOutput, speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

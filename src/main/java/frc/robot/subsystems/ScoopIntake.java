package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ScoopConfig;

public class ScoopIntake extends SubsystemBase {
  private final TalonSRX motorRotate;
  private final TalonSRX motorExtend;

  /** Creates a new Rake. */
  public ScoopIntake() {
    motorRotate = new TalonSRX(ScoopConfig.rotateID);
    motorExtend = new TalonSRX(ScoopConfig.extendID);
  }

  @Override
  public void periodic() {
  }

  public void setMotorSpeed(double speed) {
    motorRotate.set(ControlMode.PercentOutput, speed);
  }

  public void extend() {
    motorExtend.set(ControlMode.PercentOutput, ScoopConfig.motorSpeed);
  }

  public void retract() {
    motorExtend.set(ControlMode.PercentOutput, -ScoopConfig.motorSpeed);
  }

  public void stopExtend() {
    motorExtend.set(ControlMode.PercentOutput, 0);
  }
}

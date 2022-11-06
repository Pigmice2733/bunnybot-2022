// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.RakeConfig;
import frc.robot.Constants.ShuffleboardConfig;
import frc.robot.Constants.RakeConfig.RakeMode;

public class RakeNew extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(RakeConfig.leftMotorID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(RakeConfig.rightMotorID, MotorType.kBrushless);

  private final PIDController leftController = new PIDController(RakeConfig.kP, RakeConfig.kI, RakeConfig.kD);
  private final PIDController rightController = new PIDController(RakeConfig.kP, RakeConfig.kI, RakeConfig.kD);

  private final ShuffleboardTab rakeTab;
  private final NetworkTableEntry targetAngleEntry, leftAngleEntry, rightAngleEntry, leftOutputEntry, rightOutputEntry, modeEntry;

  private RakeMode mode = RakeMode.Manual;

  public RakeNew() {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    // sets encoders to report in degrees
    leftMotor.getEncoder().setPositionConversionFactor(360 * RakeConfig.gearRatio);
    rightMotor.getEncoder().setPositionConversionFactor(360 * RakeConfig.gearRatio);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftController.setSetpoint(RakeConfig.startAngle);
    rightController.setSetpoint(RakeConfig.startAngle);

    rakeTab = Shuffleboard.getTab("Rake");
    targetAngleEntry = rakeTab.add("Target Angle", RakeConfig.startAngle).getEntry();
    leftAngleEntry = rakeTab.add("Left Angle", RakeConfig.startAngle).getEntry();
    rightAngleEntry = rakeTab.add("Right Angle", RakeConfig.startAngle).getEntry();
    leftOutputEntry = rakeTab.add("Left Output", 0).getEntry();
    rightOutputEntry = rakeTab.add("Right Output", 0).getEntry();
    modeEntry = rakeTab.add("Mode", mode.toString()).getEntry();
  }

  @Override
  public void periodic() {
    if (mode == RakeMode.Automatic) 
      evaluateControllers();

    if (ShuffleboardConfig.rakePrintsEnabled) 
      updateShuffleboard();
  }

  private void evaluateControllers() {
    double leftPos = leftMotor.getEncoder().getPosition();
    double rightPos = rightMotor.getEncoder().getPosition();

    setOutputs(leftController.calculate(leftPos), rightController.calculate(rightPos));
  }

  private void updateShuffleboard() {
    targetAngleEntry.setDouble(leftController.getSetpoint());
    leftAngleEntry.setDouble(leftMotor.getEncoder().getPosition());
    rightAngleEntry.setDouble(rightMotor.getEncoder().getPosition());
  }

  public void manualDrive(double left, double right) {
    if (mode == RakeMode.Manual)
      setOutputs(left, right);
  }

  private void setOutputs(double left, double right) {
    leftMotor.set(left);
    rightMotor.set(right);

    if (ShuffleboardConfig.rakePrintsEnabled) {
      leftOutputEntry.setDouble(left);
      rightOutputEntry.setDouble(right);
    }
  }

  public void setSetpoint(double setpoint) {
    leftController.setSetpoint(setpoint);
    rightController.setSetpoint(setpoint);
  }

  public void setMode(RakeMode mode) {
    this.mode = mode;

    if (ShuffleboardConfig.rakePrintsEnabled)
      modeEntry.setString(mode.toString());
  }

  public void toggleMode() {
    if (this.mode == RakeMode.Automatic)
      setMode(RakeMode.Manual);

    if (this.mode == RakeMode.Manual)
      setMode(RakeMode.Automatic);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.RakeConfig;
import frc.robot.Constants.ShuffleboardConfig;
import frc.robot.Constants.RakeConfig.RakeMode;

public class Rake extends SubsystemBase {
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor = new CANSparkMax(RakeConfig.rightMotorID, MotorType.kBrushless);

  private final PIDController leftController = new PIDController(
      RakeConfig.kP, RakeConfig.kI, RakeConfig.kD);
  private final PIDController rightController = new PIDController(
      RakeConfig.kP, RakeConfig.kI, RakeConfig.kD);

 private double leftOutput;
 private double rightOutput;
  private final ShuffleboardTab rakeTab;
  private final NetworkTableEntry targetAngleEntry, leftAngleEntry, rightAngleEntry, leftOutputEntry, rightOutputEntry, modeEntry;
   
  private final DigitalInput topRightLimitSwitch = new DigitalInput(RakeConfig.topRightLimitSwitchID);
  private final DigitalInput topLeftLimitSwitch = new DigitalInput(RakeConfig.topLeftLimitSwitchID);
   private final DigitalInput bottomRightLimitSwitch = new DigitalInput(RakeConfig.bottomRightLimitSwitchID);
  private final DigitalInput bottomLeftLimitSwitch = new DigitalInput(RakeConfig.bottomLeftLimitSwitchID);

  public boolean GetTopRightSwitch() { return !topRightLimitSwitch.get(); }
  public boolean GetTopLeftSwitch() { return !topLeftLimitSwitch.get(); }
  public boolean GetBottomRightSwitch() { return !bottomRightLimitSwitch.get(); }
  public boolean GetBottomLeftSwitch() { return !bottomLeftLimitSwitch.get(); }


 
  private RakeMode mode = RakeMode.Manual;

  public Rake() {
    leftMotor = new CANSparkMax(20, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    // sets encoders to report in degrees
    leftMotor.getEncoder().setPositionConversionFactor(360 * RakeConfig.gearRatio);
    rightMotor.getEncoder().setPositionConversionFactor(360 * RakeConfig.gearRatio);

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
    
    if(GetTopLeftSwitch()){
      leftOutput = Math.min(0,leftOutput);
    }

    if(GetTopRightSwitch()){
      rightOutput = Math.min(0,rightOutput);
    }

    if(GetBottomLeftSwitch()){
      leftOutput = Math.max(0,leftOutput);
    }

    if(GetBottomRightSwitch()){
      rightOutput = Math.max(0,rightOutput);
    }

    leftMotor.set(leftOutput);
    rightMotor.set(rightOutput);

    if (mode == RakeMode.Automatic)
      evaluateControllers();
    
    

    if (ShuffleboardConfig.rakePrintsEnabled) 
      updateShuffleboard();

      //leftMotor.set(0.3);
  }

  private void evaluateControllers() {
    double leftPos = leftMotor.getEncoder().getPosition();
    double rightPos = rightMotor.getEncoder().getPosition();

    setOutputs(leftController.calculate(leftPos), rightController.calculate(rightPos));
    setOutputs(0, rightController.calculate(rightPos));
  }

  private void updateShuffleboard() {
    targetAngleEntry.setDouble(rightController.getSetpoint());
    leftAngleEntry.setDouble(leftMotor.getEncoder().getPosition());
    rightAngleEntry.setDouble(rightMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("TopLeftSwitch", topLeftLimitSwitch.get());
    SmartDashboard.putBoolean("BottomLeftSwitch", bottomLeftLimitSwitch.get());
    SmartDashboard.putString("Mode", mode.toString());
    }

  public void manualDrive(double left, double right) {
    // Enable rake when trigger is pressed a certain amount
    if (Math.abs(left) > 0.1 || Math.abs(right) > 0.1)
      setMode(RakeMode.Manual);

    if (mode == RakeMode.Manual)
      setOutputs(left, right);
  }

  public void manualDrive(double speed) {
    setOutputs(speed, speed);
  }

  public void setOutputs(double left, double right) {
    leftOutput = left;
    rightOutput = right;


    if (ShuffleboardConfig.rakePrintsEnabled) {
      leftOutputEntry.setDouble(left);
      rightOutputEntry.setDouble(right);
    }
  }

  public void setSetpoint(double setpoint) {
    // Set to automatic mode when auto rotate button is pressed
    setMode(RakeMode.Automatic);

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

    if (this.mode == RakeMode.limitSwitch)
      setMode(RakeMode.limitSwitch);
  }
  public RakeMode getMode(){
    return mode;
  }
}

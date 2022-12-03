// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls;
import frc.robot.Constants.RakeConfig;
import frc.robot.Constants.ShuffleboardConfig;
import frc.robot.Constants.RakeConfig.RakeMode;

public class Rake extends SubsystemBase {
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  private final PIDController leftController = new PIDController(
      RakeConfig.kP, RakeConfig.kI, RakeConfig.kD);
  private final PIDController rightController = new PIDController(
      RakeConfig.kP, RakeConfig.kI, RakeConfig.kD);

  private double leftOutput;
  private double rightOutput;
  private final ShuffleboardTab rakeTab;
  private final NetworkTableEntry targetAngleEntry, leftAngleEntry, rightAngleEntry, leftOutputEntry, rightOutputEntry, modeEntry, topLeftSwitchEntry, topRightSwitchEntry, bottomLeftSwitchEntry, bottomRightSwitchEntry;
   
  private final DigitalInput topRightLimitSwitch;
  private final DigitalInput topLeftLimitSwitch;
  private final DigitalInput bottomRightLimitSwitch;
  private final DigitalInput bottomLeftLimitSwitch;

  public boolean getTopRightSwitch() { return !topRightLimitSwitch.get(); }
  public boolean getTopLeftSwitch() { return !topLeftLimitSwitch.get(); } 
  public boolean getBottomRightSwitch() { return !bottomRightLimitSwitch.get(); }
  public boolean getBottomLeftSwitch() { return !bottomLeftLimitSwitch.get(); }
 
  private RakeMode mode = RakeMode.Manual;

  public Rake() {
    topRightLimitSwitch = new DigitalInput(RakeConfig.topRightLimitSwitchID);
    topLeftLimitSwitch = new DigitalInput(RakeConfig.topLeftLimitSwitchID);
    bottomRightLimitSwitch = new DigitalInput(RakeConfig.bottomRightLimitSwitchID);
    bottomLeftLimitSwitch = new DigitalInput(RakeConfig.bottomLeftLimitSwitchID);

    leftMotor = new CANSparkMax(RakeConfig.leftMotorID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(RakeConfig.rightMotorID, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();  

    //set encoders to default angle
    leftMotor.getEncoder().setPosition(RakeConfig.startAngle);
    rightMotor.getEncoder().setPosition(RakeConfig.startAngle);

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

    topLeftSwitchEntry = rakeTab.add("Top Left Switch", false).getEntry();
    topRightSwitchEntry = rakeTab.add("Top Left Switch", false).getEntry();
    bottomLeftSwitchEntry = rakeTab.add("Bottom Left Switch", false).getEntry();
    bottomRightSwitchEntry = rakeTab.add("Bottom Right Switch", false).getEntry();
    
    modeEntry = rakeTab.add("Mode", mode.toString()).getEntry();

    rakeTab.add("Left PID", leftController);
    rakeTab.add("Right PID", rightController);
  }

  @Override
  /**
   * Set the motor outputs based on PIDControllers in Automatic mode or other set outputs.
   * Clamp the motor outputs if limit switches are pressed.
   * Update the Shuffleboard display.
   */
  public void periodic() {
    if (mode == RakeMode.Automatic)
      evaluateControllers();

    if(getTopLeftSwitch()) leftOutput = Math.min(0,leftOutput);
    if(getTopRightSwitch()) rightOutput = Math.min(0,rightOutput);
    if(getBottomLeftSwitch()) leftOutput = Math.max(0,leftOutput);
    if(getBottomRightSwitch()) rightOutput = Math.max(0,rightOutput);
  
    leftMotor.set(leftOutput);
    rightMotor.set(rightOutput);

    if (ShuffleboardConfig.rakePrintsEnabled) 
      updateShuffleboard();

    if(Controls.instance != null)
    {
      if (Controls.instance.getRakeRotationSpeed() > 0.1)
        setMode(RakeMode.Manual);
    }
  }

  /** Determine the new values of the motor outputs based on the PIDControllers. */
  private void evaluateControllers() {
    double leftPos = leftMotor.getEncoder().getPosition();
    double rightPos = rightMotor.getEncoder().getPosition();

    setOutputs(leftController.calculate(leftPos), rightController.calculate(rightPos));
  }

  private void updateShuffleboard() {
    targetAngleEntry.setDouble(rightController.getSetpoint());
    leftAngleEntry.setDouble(leftMotor.getEncoder().getPosition());
    rightAngleEntry.setDouble(rightMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("TopLeftSwitch", topLeftLimitSwitch.get());
    SmartDashboard.putBoolean("BottomLeftSwitch", bottomLeftLimitSwitch.get());
    SmartDashboard.putString("Mode", mode.toString());

    topLeftSwitchEntry.setBoolean(getTopLeftSwitch());
    topRightSwitchEntry.setBoolean(getTopRightSwitch());
    bottomLeftSwitchEntry.setBoolean(getBottomLeftSwitch());
    bottomRightSwitchEntry.setBoolean(getBottomRightSwitch());
  }

  /**
   * Enable Manual mode if the triggers are pressed, then update the motor outputs to the input values.
   * @param left the new output for the left motor
   * @param right the new output for the right motor
   */
  public void manualDrive(double left, double right) {
    if (mode == RakeMode.Manual)
      setOutputs(left, right);
  }

   /**
   * Enable Manual mode if the triggers are pressed, then update the motor outputs to the input value.
   * @param speed the new output for the motors
   */
  public void manualDrive(double speed) {
    manualDrive(speed, speed);
  }

  public void setOutputs(double left, double right) {
    leftOutput = left;
    rightOutput = right;

    if (ShuffleboardConfig.rakePrintsEnabled) {
      leftOutputEntry.setDouble(left);
      rightOutputEntry.setDouble(right);
    }
  }
  
  /**
   * Set the rake mode to Automatic, then set the PIDControllers' setpoints to the input.
   * @param setpoint the new setpoint for the PIDControllers
   */
  public void setSetpoint(double setpoint) {
    // The buttons that call this method don't set the mode to Automatic, so it happens here.
    setMode(RakeMode.Automatic);

    leftController.setSetpoint(setpoint);
    rightController.setSetpoint(setpoint);
  }

  public void setMode(RakeMode mode) {
    this.mode = mode;

    if (ShuffleboardConfig.rakePrintsEnabled)
      modeEntry.setString(mode.toString());
  }

  public RakeMode getMode(){
    return mode;
  }
}

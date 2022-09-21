// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Dashboard;
import frc.robot.Constants.DrivetrainConfig;

public class Drivetrain extends SubsystemBase {
  private boolean enabled;

  public void enable() {
    this.enabled = true;
  }

  public void disable() {
    this.enabled = false;
  }

  public boolean isEnabled() {
    return this.enabled;
  }

  private final CANSparkMax leftDrive, rightDrive, rightFollower, leftFollower;

  private double leftDemand, rightDemand;
  private double leftPosition, rightPosition, heading; // heading is in degrees now

  private boolean boost = false;
  private boolean slow = false;

  // private Odometry odometry;

  private AHRS navx;
  private double navxTestAngle;
  private boolean navxTestPassed = false;
  private final NetworkTableEntry navxReport;

  private final NetworkTableEntry xDisplay, yDisplay, headingDisplay,
      leftEncoderDisplay, rightEncoderDisplay;

  // private Point initialPosition = Point.origin();

  public Drivetrain() {
    rightDrive = new CANSparkMax(DrivetrainConfig.frontLeftMotorPort,
        MotorType.kBrushless);
    rightFollower = new CANSparkMax(DrivetrainConfig.backLeftMotorPort,
        MotorType.kBrushless);
    leftDrive = new CANSparkMax(DrivetrainConfig.frontRightMotorPort,
        MotorType.kBrushless);
    leftFollower = new CANSparkMax(DrivetrainConfig.backRightMotorPort,
        MotorType.kBrushless);

    rightDrive.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();
    leftDrive.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();

    leftDrive.setInverted(true);
    leftFollower.follow(leftDrive);
    rightFollower.follow(rightDrive);

    // navx = new AHRS(DrivetrainConfig.navxPort);

    ShuffleboardLayout testReportLayout = Shuffleboard.getTab(Dashboard.systemsTestTabName)
        .getLayout("Drivetrain", BuiltInLayouts.kList)
        .withSize(2, 1)
        .withPosition(Dashboard.drivetrainTestPosition, 0);

    navxReport = testReportLayout.add("NavX", false).getEntry();

    // leftDrive.getEncoder().setPositionConversionFactor(DrivetrainConfig.rotationToDistanceConversion);
    // rightDrive.getEncoder().setPositionConversionFactor(DrivetrainConfig.rotationToDistanceConversion);

    setCoastMode(false);

    ShuffleboardLayout odometryLayout = Shuffleboard.getTab(Dashboard.developmentTabName)
        .getLayout("Odometry", BuiltInLayouts.kList).withSize(2, 5)
        .withPosition(Dashboard.drivetrainDisplayPosition, 0);

    xDisplay = odometryLayout.add("X", 0.0).getEntry();
    yDisplay = odometryLayout.add("Y", 0.0).getEntry();
    headingDisplay = odometryLayout.add("Heading", 0.0).getEntry();
    leftEncoderDisplay = odometryLayout.add("Left Encoder", 0).getEntry();
    rightEncoderDisplay = odometryLayout.add("Right Encoder", 0).getEntry();

    // odometry = new Odometry(new Pose(0.0, 0.0, 0.0));

    // Used to be in initialize()
    leftPosition = 0.0;
    rightPosition = 0.0;
    heading = 0.0;

    leftDrive.getEncoder().setPosition(0.0);
    rightDrive.getEncoder().setPosition(0.0);

    // odometry.set(new Pose(0.0, 0.0, heading), leftPosition, rightPosition);

    leftDemand = 0.0;
    rightDemand = 0.0;
  }

  public void init() {
    zeroHeading();
  }

  @Override
  public void periodic() {
    // this.getDistanceFromStart();
    // from updateInputs
    leftPosition = leftDrive.getEncoder().getPosition();
    rightPosition = rightDrive.getEncoder().getPosition();

    updateHeading();

    // odometry.update(leftPosition, rightPosition, heading);

    // from updateDashboard()
    // Pose currentPose = odometry.getPose();

    // xDisplay.setNumber(currentPose.getX());
    // yDisplay.setNumber(currentPose.getY());
    // headingDisplay.setNumber(currentPose.getHeading());
    leftEncoderDisplay.setNumber(leftPosition);
    rightEncoderDisplay.setNumber(rightPosition);
  }

  public void updateDashboard() {
  }

  public void updateHeading() {
    double headingDegrees = navx.getAngle();

    SmartDashboard.putNumber("Heading (Degrees)", headingDegrees);

    heading = headingDegrees;
  }

  public void updateInputs() {
  }

  public double getHeading() {
    return heading;
  }

  /*
   * public Pose getPose() {
   * return odometry.getPose();
   * }
   */

  public void boost() {
    this.boost = true;
  }

  public void stopBoost() {
    this.boost = false;
  }

  public void toggleBoost() {
    this.boost = !this.boost;
    this.slow = false;
  }

  public boolean isBoosting() {
    return this.boost;
  }

  public void slow() {
    this.slow = true;
  }

  public void stopSlow() {
    this.slow = false;
  }

  public void toggleSlow() {
    this.slow = !this.slow;
    this.boost = false;
  }

  public boolean isSlow() {
    return this.slow;
  }

  public boolean isCalibrating() {
    return this.navx.isCalibrating();
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftDemand = leftSpeed;
    rightDemand = rightSpeed;

    updateOutputs();
  }

  public void arcadeDrive(double forwardSpeed, double turnSpeed) {
    leftDemand = forwardSpeed + turnSpeed;
    rightDemand = forwardSpeed - turnSpeed;

    updateOutputs();
  }

  public void curvatureDrive(double forwardSpeed, double curvature) {
    double leftSpeed = forwardSpeed;
    double rightSpeed = forwardSpeed;

    if (Math.abs(forwardSpeed - 0.0) > 0.0001) {
      leftSpeed = forwardSpeed * (1 + (curvature * 0.5 *
          DrivetrainConfig.wheelBase));
      rightSpeed = forwardSpeed * (1 - (curvature * 0.5 *
          DrivetrainConfig.wheelBase));
    }

    leftDemand = leftSpeed;
    rightDemand = rightSpeed;

    updateOutputs();
  }

  public void swerveDrive(double forward, double strafe, double rotation_x) {
  }

  public void stop() {
    leftDemand = 0.0;
    rightDemand = 0.0;
  }

  public void updateOutputs() {
    leftDemand *= 0.75;
    rightDemand *= 0.75;

    if (slow) {
      leftDemand *= DrivetrainConfig.slowMultiplier;
      rightDemand *= DrivetrainConfig.slowMultiplier;
    }

    leftDrive.set(leftDemand);
    rightDrive.set(rightDemand);

    leftDemand = 0.0;
    rightDemand = 0.0;
  }

  public void test(double time) {
    if (time < 0.1) {
      navxTestAngle = navx.getAngle();
      navxTestPassed = false;
    }

    if (!navxTestPassed) {
      navxTestPassed = navx.getAngle() != navxTestAngle;
    }

    navxReport.setBoolean(navxTestPassed);
  }

  public void setCoastMode(boolean coasting) {
    CANSparkMax.IdleMode newMode = coasting ? IdleMode.kCoast : IdleMode.kBrake;
    leftDrive.setIdleMode(newMode);
    rightDrive.setIdleMode(newMode);
    leftFollower.setIdleMode(newMode);
    rightFollower.setIdleMode(newMode);
  }

  public void resetPose() {
    // this.odometry.set(new Pose(0, 0, getPose().getHeading()), 0.0, 0.0);
    // initialPosition = new Point(this.getPose());
  }

  /*
   * public double getDistanceFromStart() {
   * Point currentPosition = new Point(this.getPose());
   * SmartDashboard.putNumber("Left Position",
   * leftDrive.getEncoder().getPosition());
   * SmartDashboard.putNumber("Right Rotation",
   * rightDrive.getEncoder().getPosition());
   * SmartDashboard.putString("Current Position",
   * "(" + currentPosition.getX() + ", " + currentPosition.getY() + ")");
   * SmartDashboard.putString("INITIAL POSITION",
   * "(" + initialPosition.getX() + ", " + currentPosition.getY() + ")");
   * SmartDashboard.putNumber("Left Distance",
   * encoderTicksToPosition(leftDrive.getEncoder().getPosition()));
   * SmartDashboard.putNumber("Right Distance",
   * encoderTicksToPosition(rightDrive.getEncoder().getPosition()));
   * double leftDistance =
   * encoderTicksToPosition(leftDrive.getEncoder().getPosition());
   * double rightDistance =
   * encoderTicksToPosition(rightDrive.getEncoder().getPosition());
   * return (leftDistance + rightDistance) / 2.0;
   * }
   */

  private double encoderTicksToPosition(double ticks) {
    return ticks * DrivetrainConfig.wheelBase;
  }

  public void zeroHeading() {
    this.navx.reset();
    updateHeading();
  }
}

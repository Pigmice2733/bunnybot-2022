// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.Dashboard;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.ShuffleboardConfig;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax leftDrive, rightDrive, rightFollower, leftFollower;

  private double leftDemand, rightDemand;
  private double leftPosition, rightPosition, heading;

  private double speedFactor = 1;
  public boolean slowEnabled = false;

  private DifferentialDriveOdometry odometry;

  private AHRS navx;
  private double navxTestAngle;
  private boolean navxTestPassed = false;

  private final ShuffleboardTab drivetrainTab;
  private final NetworkTableEntry xPosEntry, yPosEntry, headingEntry, leftEncoderEntry, rightEncoderEntry, leftSpeedEntry, rightSpeedEntry, navxReport;

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

    leftDrive.getEncoder().setPositionConversionFactor(DrivetrainConfig.rotationToDistanceConversion);
    rightDrive.getEncoder().setPositionConversionFactor(DrivetrainConfig.rotationToDistanceConversion);

    navx = new AHRS();

    odometry = new DifferentialDriveOdometry(new Rotation2d(0));

    leftDrive.getEncoder().setPosition(0.0);
    rightDrive.getEncoder().setPosition(0.0);

    leftDemand = 0.0;
    rightDemand = 0.0;

    // Shuffleboard
    drivetrainTab = Shuffleboard.getTab("Drivetrain");

    ShuffleboardLayout testReportLayout = drivetrainTab
        .getLayout("Drivetrain", BuiltInLayouts.kList)
        .withSize(2, 1)
        .withPosition(Dashboard.drivetrainTestPosition, 0);

    navxReport = testReportLayout.add("NavX", false).getEntry();

    setCoastMode(false);

    ShuffleboardLayout odometryLayout = drivetrainTab
        .getLayout("Odometry", BuiltInLayouts.kList).withSize(2, 5)
        .withPosition(Dashboard.drivetrainDisplayPosition, 0);

    xPosEntry = odometryLayout.add("X", 0.0).getEntry();
    yPosEntry = odometryLayout.add("Y", 0.0).getEntry();
    headingEntry = odometryLayout.add("Heading", 0.0).getEntry();
    leftEncoderEntry = odometryLayout.add("Left Encoder", 0).getEntry();
    rightEncoderEntry = odometryLayout.add("Right Encoder", 0).getEntry();

    leftSpeedEntry = drivetrainTab.add("Left Speed", 0).getEntry();
    rightSpeedEntry = drivetrainTab.add("Right Speed", 0).getEntry();
  }

  public void init() {
    zeroHeading();
  }

  @Override
  public void periodic() {
    getEncoderPositions();
    updateHeading();
    updateOdometry();
  }

  void getEncoderPositions() {
    leftPosition = leftDrive.getEncoder().getPosition();
    rightPosition = rightDrive.getEncoder().getPosition();

    if (!ShuffleboardConfig.drivetrainPrintsEnabled)
      return;

    leftEncoderEntry.setNumber(leftPosition);
    rightEncoderEntry.setNumber(rightPosition);
  }

  public void updateOdometry() {
    odometry.update(new Rotation2d(heading), leftPosition, rightPosition);

    // Update Shuffleboard Outputs
    Pose2d currentPose = odometry.getPoseMeters();

    if (!ShuffleboardConfig.drivetrainPrintsEnabled)
      return;

    xPosEntry.setNumber(currentPose.getX());
    yPosEntry.setNumber(currentPose.getY());
    headingEntry.setNumber(currentPose.getRotation().getDegrees());
  }

  public void updateHeading() {
    double headingDegrees = navx.getAngle();
    heading = headingDegrees;
  }

  // Current rotation of the robot in degrees
  public double getHeading() {
    return heading;
  }

  // Current position of the robot in meters
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void enableSlow() {
    slowEnabled = true;
    speedFactor = DrivetrainConfig.slowMultiplier;
  }

  public void disableSlow() {
    slowEnabled = false;
    speedFactor = 1;
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

  public void stop() {
    leftDemand = 0.0;
    rightDemand = 0.0;

    updateOutputs();
  }

  public void updateOutputs() {
    leftDemand *= speedFactor;
    rightDemand *= speedFactor;

    leftDrive.set(leftDemand);
    rightDrive.set(rightDemand);

    if (ShuffleboardConfig.drivetrainPrintsEnabled) {
      leftSpeedEntry.setDouble(leftDemand);
      rightSpeedEntry.setDouble(rightDemand);
    }

    leftDemand = 0.0;
    rightDemand = 0.0;
  }

  public void navxTest(double time) {
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
    this.odometry.resetPosition(new Pose2d(), new Rotation2d());

    leftDrive.getEncoder().setPosition(0.0);
    rightDrive.getEncoder().setPosition(0.0);
  }

  // Distance in meters from where the robot was enabled, or resetPose() was called
  public double getDistanceFromStart() {
    double leftDistance = leftDrive.getEncoder().getPosition();
    double rightDistance = rightDrive.getEncoder().getPosition();

    return (leftDistance + rightDistance) / 2.0;
  }

  public void zeroHeading() {
    this.navx.reset();
    updateHeading();
  }
}

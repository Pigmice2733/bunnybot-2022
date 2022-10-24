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
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Units;

public class Drivetrain extends SubsystemBase {
  private final NetworkTableEntry xPosEntry, yPosEntry, headingEntry, leftOutputEntry, rightOutputEntry;

  private final CANSparkMax leftDrive = new CANSparkMax(DrivetrainConfig.leftDrivePort, MotorType.kBrushless);
  private final CANSparkMax rightDrive = new CANSparkMax(DrivetrainConfig.rightDrivePort, MotorType.kBrushless);

  private final CANSparkMax leftFollow = new CANSparkMax(DrivetrainConfig.leftFollowPort, MotorType.kBrushless);
  private final CANSparkMax rightFollow = new CANSparkMax(DrivetrainConfig.rightFollowPort, MotorType.kBrushless);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.drivetrainWidthMeters);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry();

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivetrainConfig.kS, DrivetrainConfig.kV, DrivetrainConfig.kA);

  private Pose2d pose;

  public Drivetrain() {
    leftFollow.follow(leftDrive);
    rightFollow.follow(rightDrive);

    leftDrive.setInverted(false);
    rightDrive.setInverted(true);

    leftDrive.getEncoder().setPositionConversionFactor(DrivetrainConfig.rotationToDistanceConversion);
    rightDrive.getEncoder().setPositionConversionFactor(DrivetrainConfig.rotationToDistanceConversion);
  
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");
    ShuffleboardLayout odometryLayout = drivetrainTab.getLayout("Odometry", BuiltInLayouts.kList).withSize(2, 5);

    xPosEntry = odometryLayout.add("X", 0.0).getEntry();
    yPosEntry = odometryLayout.add("Y", 0.0).getEntry();
    headingEntry = odometryLayout.add("Heading", 0.0).getEntry();

    leftOutputEntry = drivetrainTab.add("Left Output", 0).getEntry();
    rightOutputEntry = drivetrainTab.add("Right Output", 0).getEntry();
  }

  public void periodic() {
    updateOdometry();
  }

  void updateOdometry() {
    pose = odometry.update(getHeading(), getMotorSpeeds())

    if (ShuffleboardConfig.drivetrainPrintsEnabled) {
      xPosEntry.setDouble(pose.getX());
      yPosEntry.setDouble(pose.getY());
      headingEntry.setDouble(pose.getRotation().getDegrees());
    }
  }

  public Rotation2d getHeading() {
    return new Rotation2d(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getMotorSpeeds() {
    left = leftDrive.getEncoder().getVelocity();
    right = rightDrive.getEncoder().getVelocity();

    return new DifferentialDriveWheelSpeeds(left, right);
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public void tankDrive(double left, double right) {
    updateOutputs(left, right);
  }

  public void tankDriveVolts(double left, double right) {
    updateOutputs(left / 12, right / 12); // Divides by 12 to scale possible inputs between 0 and 1 (12 in max volts)
  }

  public void arcadeDrive(double forward, double turn) {
    double left = forward + turn;
    double right = forward - turn;

    updateOutputs(left, right);
  }

  public void updateOutputs(double left, double right) {
    leftDrive.set(left);
    rightDrive.set(right);

    if (ShuffleboardConfig.drivetrainPrintsEnabled) {
      leftOutputEntry.setDouble(left);
      rightOutputEntry.setDouble(right);
    }
  }

  public void stop() {
    leftDemand = 0.0;
    rightDemand = 0.0;

    updateOutputs();
  }

  public void resetOdometry() {
    odometry.resetPosition(new Pose2d(), new Rotation2d());
    navx.reset();

    leftDrive.getEncoder().resetPosition();
    rightDrive.getEncoder().resetPosition();
  }
}

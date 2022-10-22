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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax leftDrive = new CANSparkMax(DrivetrainConfig.leftDrivePort, MotorType.kBrushless);
  private CANSparkMax rightDrive = new CANSparkMax(DrivetrainConfig.rightDrivePort, MotorType.kBrushless);

  private CANSparkMax leftFollow = new CANSparkMax(DrivetrainConfig.leftFollowPort, MotorType.kBrushless);
  private CANSparkMax rightFollow = new CANSparkMax(DrivetrainConfig.rightFollowPort, MotorType.kBrushless);

  AHRS gyro = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.drivetrainWidthMeters);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry()

  public Drivetrain() {
    leftFollow.follow(leftDrive);
    rightFollow.follow(rightDrive);

    leftDrive.setInverted(false);
    rightDrive.setInverted(true);

    leftDrive.getEncoder().setPositionConversionFactor(DrivetrainConfig.rotationToDistanceConversion);
    rightDrive.getEncoder().setPositionConversionFactor(DrivetrainConfig.rotationToDistanceConversion);
  }

  public void periodic() {
    updateOdometry();
  }

  void updateOdometry() {
    odometry.update(getHeading(), )
  }

  public Rotation2d getHeading() {
    return new Rotation2d(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    left = leftDrive.getEncoder().getVelocity();
    right = rightDrive.getEncoder().getVelocity();

    return new DifferentialDriveWheelSpeeds(left, right);
  }

}

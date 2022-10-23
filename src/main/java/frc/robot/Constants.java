// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final static class Dashboard {
    public static final String systemsTestTabName = "Systems Test";
    public static final String driverTabName = "Driver";
    public static final String developmentTabName = "Development";

    public static final int drivetrainTestPosition = 6;

    public static final int deployTimestampPosition = 0;
    public static final int drivetrainDisplayPosition = 4;
  }

  public final static class DrivetrainConfig {
    public static final int leftDrivePort = 0;
    public static final int rightDrivePort = 4;
    public static final int leftFollowPort = 2;
    public static final int rightFollowPort = 3;

    public static final double axisThreshold = 0.1;
    public static final double wheelBase = 0.5; // circumference / gear ratio

    public static final double driveSpeed = 1;
    public static final double turnSpeed = 1;

    public static final double slowMultiplier = 0.25;

    public static final double gearRatio = 1; // Times motor has to rotate for wheel to rotate once
    public static final double wheelDiameterMeters = Units.inchesToMeters(1);
    public static final double rotationToDistanceConversion = (Math.PI * wheelDiameterMeters) * gearRatio; // Encoder rotations to distance moved
    public static final double drivetrainWidthMeters = Utils.inchesToMeters(1); // Distance between left and right wheels in meters

    // Path following PID
    public static final double kP = 9.95;
    public static final double kI = 0;
    public static final double kD = 0;

    // Drivetrain characterization
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // Ramsete config
    public static final double kB = 2.0;
    public static final double kZeta = 0.7;

  }

  public final static class RakeConfig {
    public static enum RakeMode {
      Automatic,
      Manual,
      Disabled
    }

    public static final int leftMotorID = 2;
    public static final int rightMotorID = 0;
    public static final double motorSpeed = 1;

    public static final double startAngle = 60;
    public static final double raiseAngle = 15;
    public static final double dispenseAngle = 125;
    public static final double intakeAngle = 0;
  }

  public final static class ScoopConfig {
    public static final int rotateID = 1;
    public static final int extendID = 2;
    public static final double motorSpeed = 1;
  }

  public final static class ConveyorConfig {
    public static final int botMotorID = 1;
    public static final int topMotorID = 2;
    public static final double speed = 0.1;

    public static final boolean botMotorInverted = false;
    public static final boolean topMotorInverted = false;
  }

  public final static class ShuffleboardConfig {
    public static final boolean drivetrainPrintsEnabled = true;
    public static final boolean rakePrintsEnabled = true;
  }
}

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
    public static final int frontLeftMotorPort = 0;
    public static final int frontRightMotorPort = 4;
    public static final int backLeftMotorPort = 2;
    public static final int backRightMotorPort = 3;

    public static final double axisThreshold = 0.1;
    public static final double wheelBase = 0.5; // circumference / gear ratio

    public static final double driveSpeed = 1;
    public static final double turnSpeed = 1;

    public static final double slowMultiplier = 0.25;

    public static final double gearRatio = 1;
    public static final double wheelDiameterMeters = 1;
    public static final double rotationToDistanceConversion = (Math.PI * wheelDiameterMeters) / gearRatio;

    public static final double maxVelocity = 1.0;
    public static final double maxAcceleration = 1.5;
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

    public static final double startAngle = 40;
    public static final double raiseAngle = 15;
    public static final double dispenseAngle = 160;
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
}

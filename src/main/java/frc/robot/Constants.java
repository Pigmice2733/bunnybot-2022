// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
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
        public static final double axisThreshold = 0.1;
        public static final double wheelBase = 0.5; // circumference / gear ratio

        public static final double driveSpeed = 1;
        public static final double turnSpeed = 1;

        public static final double slowMultiplier = 0.25;

        public static final int frontLeftMotorPort = 0;
        public static final int frontRightMotorPort = 4;
        public static final int backLeftMotorPort = 2;
        public static final int backRightMotorPort = 3;
    }

    public final static class RakeConfig {
        public static final int motorID = 2;
        public static final double motorSpeed = 1;
    }
}

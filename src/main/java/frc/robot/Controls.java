package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.RakeConfig;

public class Controls {
    XboxController driver;
    XboxController operator;

    // ALL CONTORLS ARE TEMPORARY UNTIL WE MEET WITH DRIVE TEAM

    private double threshold = DrivetrainConfig.axisThreshold;
    // if a value from a joystick is less than this, it will return 0

    public Controls(XboxController driver, XboxController operator) {
        this.driver = driver;
        this.operator = operator;
    }

    public double getDriveSpeed() {
        double joystickValue = driver.getLeftY();
        joystickValue = MathUtil.applyDeadband(-joystickValue, threshold); // Deals with stick drag

        return joystickValue * DrivetrainConfig.driveSpeed;
    }

    public double getTurnSpeed() {
        double joystickValue = driver.getRightX();
        joystickValue = MathUtil.applyDeadband(joystickValue, threshold); // Deals with stick drag

        return joystickValue * DrivetrainConfig.turnSpeed;
    }

    public double getRakeRotationSpeed() {
        double joystickValue = operator.getRightTriggerAxis() - operator.getLeftTriggerAxis();
        joystickValue = MathUtil.applyDeadband(joystickValue, threshold);
        return joystickValue * RakeConfig.motorSpeed;
    }
}

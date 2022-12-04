package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.RakeConfig;

public class Controls {
    public static Controls instance;

    XboxController driver;
    XboxController operator;

    private double threshold = Constants.axisThreshold; // If a value from a joystick is less than this, it will return 0.

    public Controls(XboxController driver, XboxController operator) {
        instance = this;
        this.driver = driver;
        this.operator = operator;
    }

    /** Return the left joystick's Y as long as it's over the threshold. */
    public double getDriveSpeed() {
        double joystickValue = driver.getLeftY();
        joystickValue = MathUtil.applyDeadband(-joystickValue, threshold); // deals with stick drag

        return joystickValue * DrivetrainConfig.driveSpeed;
    }

    /** Return the right joystick's X as long as it's over the threshold. */
    public double getTurnSpeed() {
        double joystickValue = driver.getRightX();
        joystickValue = MathUtil.applyDeadband(joystickValue, threshold); // deals with stick drag

        return joystickValue * DrivetrainConfig.turnSpeed;
    }

    /** Return the difference between the right trigger's output and the left trigger's output as long as it's over the threshold. */
    public double getRakeRotationSpeed() {
        double joystickValue = operator.getRightTriggerAxis() - operator.getLeftTriggerAxis();
        joystickValue = MathUtil.applyDeadband(joystickValue, threshold);
        return joystickValue * RakeConfig.motorSpeed;
        
    }
}

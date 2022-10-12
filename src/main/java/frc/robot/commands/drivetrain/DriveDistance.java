package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends ProfiledPIDCommand {
  private final Drivetrain drivetrain;
  private final double maxError = 0.05;
  private final double maxVelocity = 1.0;

  public DriveDistance(Drivetrain drivetrain, double distance) {
    super(
        new ProfiledPIDController(
            1.25,
            0.0005,
            0.005,
            new TrapezoidProfile.Constraints(DrivetrainConfig.maxVelocity, DrivetrainConfig.maxAcceleration)),
        drivetrain::getDistanceFromStart,
        Math.abs(distance),
        (output, setpoint) -> {
          SmartDashboard.putNumber("Distance", drivetrain.getDistanceFromStart());
          output = MathUtil.clamp(output, -0.40, 0.40);
          drivetrain.arcadeDrive(output * (distance < 0 ? -1 : 1), 0);
        },
        drivetrain);

    this.drivetrain = drivetrain;

    getController().setTolerance(maxError, maxVelocity);
  }

  @Override
  public void initialize() {
    this.drivetrain.resetPose();
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
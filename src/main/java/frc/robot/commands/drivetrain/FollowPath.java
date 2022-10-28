package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class FollowPath extends RamseteCommand {
    private Drivetrain drivetrain;

    public FollowPath(Drivetrain drivetrain, Trajectory trajectory) {
        super(
            trajectory,
            drivetrain::getPose,
            new RamseteController(DrivetrainConfig.kB, DrivetrainConfig.kZeta),
            drivetrain.getFeedForward(),
            drivetrain.getKinematics(),
            drivetrain::getWheelSpeeds,
            new PIDController(DrivetrainConfig.kP, DrivetrainConfig.kI, DrivetrainConfig.kD), // Left
            new PIDController(DrivetrainConfig.kP, DrivetrainConfig.kI, DrivetrainConfig.kD), // Right
            drivetrain::tankDriveVolts,
            drivetrain
        );
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.resetOdometry();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0.0, 0.0);
    }
}
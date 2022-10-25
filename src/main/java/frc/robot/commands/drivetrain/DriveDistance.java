package frc.robot.commands.drivetrain;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class DriveDistance extends CommandBase {
    public FollowPath followPath;

    public DriveDistance(Drivetrain drivetrain, double distance) {
        TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5);
        config.setKinematics(drivetrain.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(),
            List.of(new Translation2d(distance, 0)),
            new Pose2d(distance, 0, new Rotation2d()),
            config
            );


        followPath = new FollowPath(drivetrain, trajectory);

        addRequirements(drivetrain);
    }

    @Override
    public boolean isFinished() {
        return followPath.isFinished();
    }
}
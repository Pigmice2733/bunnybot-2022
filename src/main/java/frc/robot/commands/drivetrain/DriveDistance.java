package com.pigmice.frc.robot.commands.drivetrain;

import com.pigmice.frc.robot.Constants.DrivetrainConfig;
import com.pigmice.frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;

public class DriveDistance extends CommandBase {
    public FollowPath followPath;

    public DriveDistance(Drivetrain drivetrain, double distance) {
        TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5);
        config.setKinematics(drivetrain.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTeajectory(
            Arrays.asList(new Pose2d, new Pose2d(distance, 0, new Rotation2d()),
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
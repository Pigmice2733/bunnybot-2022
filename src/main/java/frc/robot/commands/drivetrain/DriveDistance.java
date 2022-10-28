package frc.robot.commands.drivetrain;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class DriveDistance extends CommandBase {
    public FollowPath followPathCommand;

    public DriveDistance(Drivetrain drivetrain, double distance) {
        TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5);
        config.setKinematics(drivetrain.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(), new Pose2d(distance, 0, new Rotation2d())),
            config
        );
        followPathCommand = new FollowPath(drivetrain, trajectory);
        
        SmartDashboard.putBoolean("Command Done", false);

        addRequirements(drivetrain);
    }

    @Override 
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("Command Done", true);
        return followPathCommand.isFinished();
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.drivetrain.FollowPath;
import frc.robot.subsystems.Drivetrain;

public class TestPath extends CommandBase {
  /**
   * Autonomous routine. Follow the path specified in generateTrajectory.
   * @param drivetrain a drivetrain subsystem
   */
  public TestPath(Drivetrain drivetrain) {
      TrajectoryConfig config = new TrajectoryConfig(1.7, 0.7);
      config.setKinematics(drivetrain.getKinematics());
  
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          List.of(new Pose2d(), new Pose2d(2, -2, new Rotation2d(-45)), new Pose2d(2.5, 0, new Rotation2d(90))),
          config);
  
      FollowPath pathCommand = new FollowPath(drivetrain, trajectory);
      CommandScheduler.getInstance().schedule(pathCommand);

      addRequirements(drivetrain);
  }
}

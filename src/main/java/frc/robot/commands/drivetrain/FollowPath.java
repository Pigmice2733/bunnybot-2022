package com.pigmice.frc.robot.commands.drivetrain;

import com.pigmice.frc.robot.Constants.DrivetrainConfig;
import com.pigmice.frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class FollowPath extends RamseteCommand {
    private Drivetrain drivetrain;

    public Ramsete(Drivetrain drivetrain, Trajectory trajectory) {
        super(
            trajectory,
            drivetrain::getPose,
            new RamseteController(DrivetrainConfig.kB, DrivetrainConfig.kZeta),
            drivetrain.getFeedForward(),
            drivetrain.getKinematics(),
            drivetrain::getMotorSpeeds,
            new PIDController(DrivetrainConfig.kP, DrivetrainConfig.kI, DrivetrainConfig.kD), // Left
            new PIDController(DrivetrainConfig.kP, DrivetrainConfig.kI, DrivetrainConfig.kD), // Right
            drivetrain::tankDriveVolts,
            drivetrain
        );
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0.0, 0.0);
    }
}
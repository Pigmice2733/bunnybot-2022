package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final double left;
    private final double right;

    public TankDrive(Drivetrain drivetrain, DoubleSupplier left, DoubleSupplier right) {
        this.drivetrain = drivetrain;
        this.left = left.getAsDouble();
        this.right = right.getAsDouble();

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.tankDrive(left, right);
    }
}
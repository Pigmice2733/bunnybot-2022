package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
    private Drivetrain drivetrain;
    private DoubleSupplier forward, rotate;

    public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier forward, DoubleSupplier rotate) {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.rotate = rotate;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(forward.getAsDouble(), rotate.getAsDouble());
        System.out.println(forward.getAsDouble() + " " + rotate.getAsDouble());
    }
}

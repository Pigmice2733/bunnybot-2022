package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;

public class ExampleCommand extends CommandBase {
    private ExampleSubsystem exampleSubsystem;

    public ExampleCommand(ExampleSubsystem exampleSubsystem) {
        this.exampleSubsystem = exampleSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

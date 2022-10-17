package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Rake;

public class RotateRakeManual extends CommandBase {
  private final Rake rake;
  private DoubleSupplier speed;

  public RotateRakeManual(DoubleSupplier speed, Rake rake) {
    this.rake = rake;
    this.speed = speed;
  }

  @Override
  public void execute() {
    rake.setSpeedManual(speed.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

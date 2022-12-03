package frc.robot.commands.rake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Rake;

public class RotateRakeManual extends CommandBase {
  private final Rake rake;
  private DoubleSupplier speed;

  /**
   * Rotate the rake at the speed given.
   * @param speed The speed input calculated in the rake subsystem.
   *              This will move the rake up if the right trigger is pressed or down if the left trigger is pressed.
   * @param rake a rake subsystem
   */
  public RotateRakeManual(DoubleSupplier speed, Rake rake) {
    this.rake = rake;
    this.speed = speed;

    addRequirements(rake);
  }

  @Override
  public void execute() {
    rake.manualDrive(speed.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

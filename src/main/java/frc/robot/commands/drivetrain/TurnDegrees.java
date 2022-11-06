package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class TurnDegrees extends PIDCommand { 
  private Drivetrain drivetrain;
  private double rotation;

  public TurnDegrees(Drivetrain drivetrain, double rotation) {
    super(
      new PIDController(0.006, 0, 0.0001), 
          drivetrain::getHeadingDegrees,
          (rotation + drivetrain.getHeadingDegrees()) % 360, 
          (output) -> { drivetrain.arcadeDrive(0, output); },
          drivetrain
    );
    this.m_controller.setTolerance(3, 0.1);
    this.m_controller.enableContinuousInput(0, 360);

    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.rotation = rotation;
  } 

  @Override
  public void initialize() {
    getController().setSetpoint((rotation + drivetrain.getHeadingDegrees()) % 360);
  }

  @Override
  public boolean isFinished() {
    return this.m_controller.atSetpoint();
  }
}

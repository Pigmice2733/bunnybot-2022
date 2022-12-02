package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class TurnDegrees extends PIDCommand { 
  private Drivetrain drivetrain;
  private double rotation;

  public TurnDegrees(Drivetrain drivetrain, double rotation) {
    super(
      new PIDController(DrivetrainConfig.turnP, DrivetrainConfig.turnI, DrivetrainConfig.turnD), 
        drivetrain::getHeadingDegrees,
        (rotation + drivetrain.getHeadingDegrees()) % 360, 
        (output) -> { drivetrain.arcadeDrive(0, output); },
        drivetrain
    );
    getController().setTolerance(3, 1);
    getController().enableContinuousInput(0, 360);

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
    return getController().atSetpoint();
  }
}

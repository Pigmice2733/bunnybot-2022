package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class TurnDegrees extends ProfiledPIDCommand { 
  private Drivetrain drivetrain;
  private double rotation;

  /**
   * Use PIDControllers to turn the specified number of degrees.
   * @param drivetrain a drivetrain subsystem
   * @param rotation The number of degrees to turn. The robot will turn right if this is positive or left if this is negative.
   */
  public TurnDegrees(Drivetrain drivetrain, double rotation) {
    super(
      RobotContainer.turnDegreesController, 
        drivetrain::getHeadingDegrees,
        rotation, 
        (output,setpoint) -> { drivetrain.arcadeDrive(0, output); },
        drivetrain
    );
    getController().setTolerance(3, 1);
    getController().enableContinuousInput(0, 360);
    drivetrain.resetOdometry();
    //Shuffleboard.getTab("Drivetrain").add("Drive Distance PID", getController());

    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.rotation = rotation;
  } 

  @Override
  public void initialize() {
    // getController().setSetpoint((rotation + drivetrain.getHeadingDegrees()) % 360);
    // System.out.println((rotation + drivetrain.getHeadingDegrees()) % 360);
    //drivetrain.resetOdometry();
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

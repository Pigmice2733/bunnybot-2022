package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class TurnDegrees extends PIDCommand { 
  /**
   * Use profiled PID to turn the specified number of degrees.
   * @param drivetrain a drivetrain subsystem
   * @param rotation The number of degrees to turn. The robot will turn right if this is positive or left if this is negative.
   */
  public TurnDegrees(Drivetrain drivetrain, double rotation) {
    super(
        RobotContainer.turnDegreesController, 
        drivetrain::getHeadingDegrees,
        rotation, 
        (output) -> { drivetrain.arcadeDrive(0, output); },
        drivetrain
    );
    getController().setTolerance(3.0, 1.0);
    getController().enableContinuousInput(0.0, 360.0);
    drivetrain.resetOdometry();
    //Shuffleboard.getTab("Drivetrain").add("Drive Distance PID", getController());

    addRequirements(drivetrain);
  } 

  @Override
  public void initialize() {
    // getController().setSetpoint((rotation + drivetrain.getHeadingDegrees()) % 360);
    // System.out.println((rotation + drivetrain.getHeadingDegrees()) % 360);
    // drivetrain.resetOdometry();
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

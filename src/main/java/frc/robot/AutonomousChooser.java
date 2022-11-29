package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoRoutines.AutoDispense;
import frc.robot.commands.AutoRoutines.DriveAndDispense;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Rake;

public class AutonomousChooser {
  // private final ShuffleboardTab autoTab;
  // public final SendableChooser<Command> chooser;
  // private final Rake rake;
  // private final Drivetrain drivetrain;

  // public AutonomousChooser() {
  //   drivetrain = new Drivetrain();
  //   rake = new Rake();

  //   chooser = new SendableChooser<Command>();
  //   chooser.setDefaultOption("none", null);
  //   chooser.addOption("AutoDispense", new AutoDispense(drivetrain, rake));
  //   chooser.addOption("DriveAndDispense", new DriveAndDispense(drivetrain, rake));

  //   autoTab = Shuffleboard.getTab("Autonomous");
  //   autoTab.add("Autonomous Command", chooser);
  // }
}
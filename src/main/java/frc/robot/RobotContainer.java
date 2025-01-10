package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.helpers.CCSparkMax;
import frc.robot.maps.Constants;
import frc.robot.subsystems.DriveTrain.SwerveDriveSubsystem;

public class RobotContainer {

  SwerveDriveSubsystem drive;

  /*
   * Initialize controllers.
   */
  CommandXboxController controller1 = new CommandXboxController(0);
  CommandXboxController controller2 = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        drive = new SwerveDriveSubsystem(CCSparkMax::new);

        break;

      case SIM:
        drive = new SwerveDriveSubsystem(CCSparkSim::new);
        break;

      case REPLAY:
        break;
    }
  }
}

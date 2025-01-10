package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.helpers.CCSparkMax;
import frc.robot.helpers.CCSparkSim;
import frc.robot.maps.Constants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleIOHardware;
import frc.robot.subsystems.swervedrive.SwerveModuleIOSim;

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
        drive = new SwerveDriveSubsystem(CCSparkMax::new, SwerveModuleIOHardware::new);

        break;

      case SIM:
        drive = new SwerveDriveSubsystem(CCSparkSim::new, SwerveModuleIOSim::new);
        break;

      case REPLAY:
        break;
    }
  }
}

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.CCSparkMax;
import frc.helpers.CCSparkSim;
import frc.maps.Constants;
import frc.robot.subsystems.algae.AlgaeIOHardware;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmIOHardware;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleIOHardware;
import frc.robot.subsystems.swervedrive.SwerveModuleIOSim;
import frc.robot.subsystems.wrist.WristIOHardware;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RobotContainer {

  AlgaeSubsystem algae;
  ArmSubsystem arm;
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;
  SwerveDriveSubsystem swerve;
  WristSubsystem wrist;


  /*
   * Initialize controllers.
   */
  CommandXboxController controller1 = new CommandXboxController(0);
  CommandXboxController controller2 = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        swerve = SwerveDriveSubsystem.getInstance(CCSparkMax::new, SwerveModuleIOHardware::new);
        algae = AlgaeSubsystem.getInstance(CCSparkMax::new, AlgaeIOHardware::new);
        arm = ArmSubsystem.getInstance(CCSparkMax::new, ArmIOHardware::new);
        elevator = ElevatorSubsystem.getInstance(CCSparkMax::new, ElevatorIOHardware::new);
        intake = IntakeSubsystem.getInstance(CCSparkMax::new, IntakeIOHardware::new);
        wrist = WristSubsystem.getInstance(CCSparkMax::new, WristIOHardware::new); 


        break;

      case SIM:
        swerve = SwerveDriveSubsystem.getInstance(CCSparkSim::new, SwerveModuleIOSim::new);
        algae = AlgaeSubsystem.getInstance(CCSparkSim::new, AlgaeIOSim::new);
        arm = ArmSubsystem.getInstance(CCSparkSim::new, ArmIOSim::new);
        elevator = ElevatorSubsystem.getInstance(CCSparkSim::new, ElevatorIOSim::new);
        intake = IntakeSubsystem.getInstance(CCSparkSim::new, IntakeIOSim::new);
        wrist = WristSubsystem.getInstance(CCSparkSim::new, WristIOSim::new); 



        break;

      case REPLAY:
        break;
    }
  }
}

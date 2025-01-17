package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.CCMotorReplay;
import frc.helpers.CCSparkMax;
import frc.helpers.CCSparkSim;
import frc.helpers.vision.PhotonVision;
import frc.maps.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Algae.AlgaeIOHardware;
import frc.robot.subsystems.Algae.AlgaeIOReplay;
import frc.robot.subsystems.Algae.AlgaeIOSim;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Arm.ArmIOHardware;
import frc.robot.subsystems.Arm.ArmIOReplay;
import frc.robot.subsystems.Arm.ArmIOSim;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Elevator.ElevatorIOHardware;
import frc.robot.subsystems.Elevator.ElevatorIOReplay;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeIOHardware;
import frc.robot.subsystems.Intake.IntakeIOReplay;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleIOHardware;
import frc.robot.subsystems.swervedrive.SwerveModuleIOReplay;
import frc.robot.subsystems.swervedrive.SwerveModuleIOSim;
import frc.robot.subsystems.Wrist.WristIOHardware;
import frc.robot.subsystems.Wrist.WristIOReplay;
import frc.robot.subsystems.Wrist.WristIOSim;
import frc.robot.subsystems.Wrist.WristSubsystem;

public class RobotContainer {

  AlgaeSubsystem algae;
  ArmSubsystem arm;
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;
  SwerveDriveSubsystem swerve;
  WristSubsystem wrist;

  PhotonVision photonvision;
  Superstructure superstructure;

  /*
   * Initialize controllers.
   */
  CommandXboxController primaryController = new CommandXboxController(0);
  CommandXboxController secondaryController = new CommandXboxController(1);

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

        photonvision = PhotonVision.getInstance();

        superstructure = Superstructure.getInstance();

        break;

      case SIM:
        swerve = SwerveDriveSubsystem.getInstance(CCSparkSim::new, SwerveModuleIOSim::new);
        algae = AlgaeSubsystem.getInstance(CCSparkSim::new, AlgaeIOSim::new);
        arm = ArmSubsystem.getInstance(CCSparkSim::new, ArmIOSim::new);
        elevator = ElevatorSubsystem.getInstance(CCSparkSim::new, ElevatorIOSim::new);
        intake = IntakeSubsystem.getInstance(CCSparkSim::new, IntakeIOSim::new);
        wrist = WristSubsystem.getInstance(CCSparkSim::new, WristIOSim::new);

        photonvision = PhotonVision.getInstance();

        superstructure = Superstructure.getInstance();

        break;

      case REPLAY:
        swerve = SwerveDriveSubsystem.getInstance(CCMotorReplay::new, SwerveModuleIOReplay::new);
        algae = AlgaeSubsystem.getInstance(CCMotorReplay::new, AlgaeIOReplay::new);
        arm = ArmSubsystem.getInstance(CCMotorReplay::new, ArmIOReplay::new);
        elevator = ElevatorSubsystem.getInstance(CCMotorReplay::new, ElevatorIOReplay::new);
        intake = IntakeSubsystem.getInstance(CCMotorReplay::new, IntakeIOReplay::new);
        wrist = WristSubsystem.getInstance(CCMotorReplay::new, WristIOReplay::new);

        superstructure = Superstructure.getInstance();

        break;
    }

    RobotState.getInstance().robotStateInit(swerve, algae, arm, elevator, intake, wrist);
    RobotState.getInstance().poseInit();
    RobotState.getInstance().moduleEncodersInit();
    RobotState.getInstance().dashboardInit();
  }
}

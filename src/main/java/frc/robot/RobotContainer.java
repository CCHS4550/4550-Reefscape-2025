package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.CCMotorReplay;
import frc.helpers.CCSparkMax;
import frc.helpers.CCSparkSim;
import frc.helpers.vision.PhotonVision;
import frc.maps.Constants;
import frc.robot.controlschemes.autoMechScheme;
import frc.robot.controlschemes.teleOpMechScheme;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae.AlgaeIOHardware;
import frc.robot.subsystems.algae.AlgaeIOReplay;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmIOHardware;
import frc.robot.subsystems.arm.ArmIOReplay;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.elevator.ElevatorIOReplay;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOReplay;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleIOHardware;
import frc.robot.subsystems.swervedrive.SwerveModuleIOReplay;
import frc.robot.subsystems.swervedrive.SwerveModuleIOSim;
import frc.robot.subsystems.wrist.WristIOHardware;
import frc.robot.subsystems.wrist.WristIOReplay;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RobotContainer {

  AlgaeSubsystem algae;
  ArmSubsystem arm;
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;
  SwerveDriveSubsystem swerve;
  WristSubsystem wrist;
  ClimberSubsystem climber;

  PhotonVision photonvision;
  Superstructure superstructure;

  /*
   * Initialize controllers.
   */
  CommandXboxController primaryController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    teleOpMechScheme.configure(intake, arm, elevator, wrist, algae, climber, 2);
    autoMechScheme.configure(intake, arm, elevator, wrist, algae, superstructure, 3);
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

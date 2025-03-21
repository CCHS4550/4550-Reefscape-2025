package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.controlschemes.MechanismScheme;
import frc.robot.controlschemes.SwerveDriveScheme;
import frc.robot.controlschemes.TestingScheme;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae.AlgaeIOReplay;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.climber.ClimberIOHardware;
import frc.robot.subsystems.climber.ClimberIOReplay;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOReplay;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.superstructure.arm.ArmIOHardware;
import frc.robot.subsystems.superstructure.arm.ArmIOReplay;
import frc.robot.subsystems.superstructure.arm.ArmIOSim;
import frc.robot.subsystems.superstructure.arm.ArmSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOHardware;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOReplay;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.wrist.WristIOHardware;
import frc.robot.subsystems.superstructure.wrist.WristIOReplay;
import frc.robot.subsystems.superstructure.wrist.WristIOSim;
import frc.robot.subsystems.superstructure.wrist.WristSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleIOHardware;
import frc.robot.subsystems.swervedrive.SwerveModuleIOReplay;
import frc.robot.subsystems.swervedrive.SwerveModuleIOSim;
import frc.robot.vision.PhotonVisionAprilTag;
import frc.robot.vision.PhotonVisionReplay;
import frc.robot.vision.PhotonVisionSim;
import frc.robot.vision.VisionIO;
import frc.util.BlinkinLEDController;
import frc.util.BlinkinLEDController.BlinkinPattern;
import frc.util.maps.Constants;
import frc.util.motorcontroller.CCMotorReplay;
import frc.util.motorcontroller.CCSparkMax;
import frc.util.motorcontroller.CCSparkSim;

public class RobotContainer {

  private final AlgaeSubsystem algae;
  private final ArmSubsystem arm;
  private final ClimberSubsystem climber;
  private final ElevatorSubsystem elevator;
  private final IntakeSubsystem intake;
  private final SwerveDriveSubsystem swerve;
  private final WristSubsystem wrist;

  private final VisionIO vision;

  private final Superstructure superstructure;

  /*
   * Initialize controllers.
   */
  CommandXboxController primaryController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        System.out.println("Creating Real Robot.");
        algae = new AlgaeSubsystem(CCMotorReplay::new, AlgaeIOReplay::new);
        arm = new ArmSubsystem(CCSparkMax::new, ArmIOHardware::new);
        climber = new ClimberSubsystem(CCSparkMax::new, ClimberIOHardware::new);
        elevator = new ElevatorSubsystem(CCSparkMax::new, ElevatorIOHardware::new);
        intake = new IntakeSubsystem(CCSparkMax::new, IntakeIOHardware::new);
        swerve = new SwerveDriveSubsystem(CCSparkMax::new, SwerveModuleIOHardware::new);
        wrist = new WristSubsystem(CCSparkMax::new, WristIOHardware::new);

        vision = new PhotonVisionAprilTag();

        superstructure = new Superstructure(algae, arm, climber, elevator, intake, swerve, wrist);

        BlinkinLEDController.getInstance().setPattern(BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

        break;

      case SIM:
        System.out.println("Creating Simulated Robot.");
        algae = new AlgaeSubsystem(CCSparkSim::new, AlgaeIOSim::new);
        arm = new ArmSubsystem(CCSparkSim::new, ArmIOSim::new);
        climber = new ClimberSubsystem(CCSparkSim::new, ClimberIOSim::new);
        elevator = new ElevatorSubsystem(CCSparkSim::new, ElevatorIOSim::new);
        intake = new IntakeSubsystem(CCSparkSim::new, IntakeIOSim::new);
        swerve = new SwerveDriveSubsystem(CCSparkSim::new, SwerveModuleIOSim::new);
        wrist = new WristSubsystem(CCSparkSim::new, WristIOSim::new);

        vision = new PhotonVisionSim();

        superstructure = new Superstructure(algae, arm, climber, elevator, intake, swerve, wrist);

        break;

      case REPLAY:
        System.out.println("Creating Replay Robot.");
        algae = new AlgaeSubsystem(CCMotorReplay::new, AlgaeIOReplay::new);
        arm = new ArmSubsystem(CCMotorReplay::new, ArmIOReplay::new);
        climber = new ClimberSubsystem(CCMotorReplay::new, ClimberIOReplay::new);
        elevator = new ElevatorSubsystem(CCMotorReplay::new, ElevatorIOReplay::new);
        intake = new IntakeSubsystem(CCMotorReplay::new, IntakeIOReplay::new);
        swerve = new SwerveDriveSubsystem(CCMotorReplay::new, SwerveModuleIOReplay::new);
        wrist = new WristSubsystem(CCMotorReplay::new, WristIOReplay::new);

        vision = new PhotonVisionReplay();

        superstructure = new Superstructure(algae, arm, climber, elevator, intake, swerve, wrist);

        break;
      default:
        System.out.println("Creating Default Robot. (Replay)");
        algae = new AlgaeSubsystem(CCMotorReplay::new, AlgaeIOReplay::new);
        arm = new ArmSubsystem(CCMotorReplay::new, ArmIOReplay::new);
        climber = new ClimberSubsystem(CCMotorReplay::new, ClimberIOReplay::new);
        elevator = new ElevatorSubsystem(CCMotorReplay::new, ElevatorIOReplay::new);
        intake = new IntakeSubsystem(CCMotorReplay::new, IntakeIOReplay::new);
        swerve = new SwerveDriveSubsystem(CCMotorReplay::new, SwerveModuleIOReplay::new);
        wrist = new WristSubsystem(CCMotorReplay::new, WristIOReplay::new);

        vision = new PhotonVisionReplay();

        superstructure = new Superstructure(algae, arm, climber, elevator, intake, swerve, wrist);
    }

    RobotState.getInstance()
        .robotStateInit(
            algae, arm, climber, elevator, intake, swerve, wrist, vision, superstructure);

    /** Configure controls. */
    switch (Constants.currentMode) {
      case REAL:
        SwerveDriveScheme.configure(
            algae,
            arm,
            climber,
            elevator,
            intake,
            swerve,
            wrist,
            superstructure,
            vision,
            primaryController);
        MechanismScheme.configure(
            algae, arm, climber, elevator, intake, swerve, wrist, superstructure, 1);
        // TestingScheme.configure(
        //     algae,
        //     arm,
        //     climber,
        //     elevator,
        //     intake,
        //     swerve,
        //     wrist,
        //     superstructure,
        //     vision,
        //     primaryController);

        // CharacterizationScheme.configure(
        //     algae,
        //     arm,
        //     climber,
        //     elevator,
        //     intake,
        //     swerve,
        //     wrist,
        //     superstructure,
        //     vision,
        //     primaryController);

        break;

      case SIM:
        SwerveDriveScheme.configure(
            algae,
            arm,
            climber,
            elevator,
            intake,
            swerve,
            wrist,
            superstructure,
            vision,
            primaryController);
        TestingScheme.configure(
            algae,
            arm,
            climber,
            elevator,
            intake,
            swerve,
            wrist,
            superstructure,
            vision,
            primaryController);
        break;

      case REPLAY:
        break;
    }
  }
}

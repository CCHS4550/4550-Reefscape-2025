package frc.robot;

import com.sun.org.apache.bcel.internal.generic.NamedAndTyped;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.CCMotorReplay;
import frc.helpers.CCSparkMax;
import frc.helpers.CCSparkSim;
import frc.helpers.vision.PhotonVisionReplay;
import frc.helpers.vision.PhotonVisionSim;
import frc.helpers.vision.VisionIO;
import frc.maps.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.controlschemes.*;
import frc.robot.subsystems.algae.AlgaeIOReplay;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmIOHardware;
import frc.robot.subsystems.arm.ArmIOReplay;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberIOReplay;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOReplay;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
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
  ClimberSubsystem climber;
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;
  SwerveDriveSubsystem swerve;
  WristSubsystem wrist;

  VisionIO vision;
  //  Superstructure superstructure;

  /*
   * Initialize controllers.
   */
  CommandXboxController primaryController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // teleOpMechScheme.configure(intake, arm, elevator, wrist, algae, climber, 1);
    // autoMechScheme.configure(intake, arm, elevator, wrist, algae, superstructure, 2);

    switch (Constants.currentMode) {
      case REAL:
        System.out.println("Creating Real Robot.");
        swerve = SwerveDriveSubsystem.getInstance(CCSparkMax::new, SwerveModuleIOHardware::new);
        algae = AlgaeSubsystem.getInstance(CCMotorReplay::new, AlgaeIOReplay::new);
        arm = ArmSubsystem.getInstance(CCSparkMax::new, ArmIOHardware::new);
        climber = ClimberSubsystem.getInstance(CCMotorReplay::new, ClimberIOReplay::new);
        elevator = ElevatorSubsystem.getInstance(CCMotorReplay::new, ElevatorIOReplay::new);
        intake = IntakeSubsystem.getInstance(CCMotorReplay::new, IntakeIOReplay::new);
        wrist = WristSubsystem.getInstance(CCSparkMax::new, WristIOHardware::new);

        // vision = PhotonVisionAprilTag.getInstance();

        // superstructure = Superstructure.getInstance();

        break;

      case SIM:
        System.out.println("Creating Simulated Robot.");
        swerve = SwerveDriveSubsystem.getInstance(CCSparkSim::new, SwerveModuleIOSim::new);
        algae = AlgaeSubsystem.getInstance(CCSparkSim::new, AlgaeIOSim::new);
        arm = ArmSubsystem.getInstance(CCSparkSim::new, ArmIOSim::new);
        climber = ClimberSubsystem.getInstance(CCSparkMax::new, ClimberIOSim::new);
        elevator = ElevatorSubsystem.getInstance(CCSparkSim::new, ElevatorIOSim::new);
        intake = IntakeSubsystem.getInstance(CCSparkSim::new, IntakeIOSim::new);
        wrist = WristSubsystem.getInstance(CCSparkSim::new, WristIOSim::new);

        vision = PhotonVisionSim.getInstance();

        // superstructure = Superstructure.getInstance();

        break;

      case REPLAY:
        System.out.println("Creating Replay Robot.");
        swerve = SwerveDriveSubsystem.getInstance(CCMotorReplay::new, SwerveModuleIOReplay::new);
        algae = AlgaeSubsystem.getInstance(CCMotorReplay::new, AlgaeIOReplay::new);
        arm = ArmSubsystem.getInstance(CCMotorReplay::new, ArmIOReplay::new);
        climber = ClimberSubsystem.getInstance(CCSparkMax::new, ClimberIOReplay::new);
        elevator = ElevatorSubsystem.getInstance(CCMotorReplay::new, ElevatorIOReplay::new);
        intake = IntakeSubsystem.getInstance(CCMotorReplay::new, IntakeIOReplay::new);
        wrist = WristSubsystem.getInstance(CCMotorReplay::new, WristIOReplay::new);

        vision = PhotonVisionReplay.getInstance();

        // superstructure = Superstructure.getInstance();

        break;
    }

    RobotState.getInstance().robotStateInit(swerve, algae, arm, elevator, intake, wrist, vision);
    RobotState.getInstance().poseInit();
    RobotState.getInstance().moduleEncodersInit();
    RobotState.getInstance().dashboardInit();

    SwerveDriveScheme.configure(swerve, primaryController);
    // CharacterizationScheme.configure(
    //     swerve, algae, arm, elevator, intake, wrist, primaryController);
  }

  
}

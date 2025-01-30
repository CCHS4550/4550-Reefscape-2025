package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.CCMotorReplay;
import frc.helpers.CCSparkMax;
import frc.helpers.CCSparkSim;
import frc.maps.Constants;
import frc.robot.controlschemes.CharacterizationScheme;
import frc.robot.controlschemes.SimulationScheme;
import frc.robot.controlschemes.SwerveDriveScheme;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae.AlgaeIOHardware;
import frc.robot.subsystems.algae.AlgaeIOReplay;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmIOHardware;
import frc.robot.subsystems.arm.ArmIOReplay;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberIOHardware;
import frc.robot.subsystems.climber.ClimberIOReplay;
import frc.robot.subsystems.climber.ClimberIOSim;
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
  ClimberSubsystem climber;
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;
  SwerveDriveSubsystem swerve;
  WristSubsystem wrist;

  Superstructure superstructure;

  /*
   * Initialize controllers.
   */
  CommandXboxController primaryController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        System.out.println("Creating Real Robot.");
        algae = new AlgaeSubsystem(CCSparkMax::new, AlgaeIOHardware::new);
        arm = new ArmSubsystem(CCSparkMax::new, ArmIOHardware::new);
        climber = new ClimberSubsystem(CCSparkMax::new, ClimberIOHardware::new);
        elevator = new ElevatorSubsystem(CCSparkMax::new, ElevatorIOHardware::new);
        intake = new IntakeSubsystem(CCSparkMax::new, IntakeIOHardware::new);
        swerve = new SwerveDriveSubsystem(CCSparkMax::new, SwerveModuleIOHardware::new);
        wrist = new WristSubsystem(CCSparkMax::new, WristIOHardware::new);

        superstructure = new Superstructure(algae, arm, climber, elevator, intake, swerve, wrist);

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

        superstructure = new Superstructure(algae, arm, climber, elevator, intake, swerve, wrist);

        break;
    }
    RobotState.getInstance().robotStateInit(algae, arm, climber, elevator, intake, swerve, wrist);
    RobotState.getInstance().poseInit();
    RobotState.getInstance().moduleEncodersInit();
    RobotState.getInstance().dashboardInit();

    switch (Constants.currentMode) {
      case REAL:
        SwerveDriveScheme.configure(
            algae, arm, climber, elevator, intake, swerve, wrist, primaryController);
        CharacterizationScheme.configure(
            algae, arm, climber, elevator, intake, swerve, wrist, primaryController);

        break;

      case SIM:
        SwerveDriveScheme.configure(
            algae, arm, climber, elevator, intake, swerve, wrist, primaryController);
        SimulationScheme.configure(
            algae, arm, climber, elevator, intake, swerve, wrist, primaryController);
        break;

      case REPLAY:
        break;
    }
  }
}

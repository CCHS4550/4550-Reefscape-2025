package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.CCMotorReplay;
import frc.helpers.CCSparkMax;
import frc.helpers.CCSparkSim;
import frc.maps.Constants;
import frc.robot.controlschemes.*;
import frc.robot.subsystems.Superstructure;
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

  /*
   * Initialize controllers.
   */
  CommandXboxController primaryController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        System.out.println("Creating Real Robot.");
        SwerveDriveSubsystem.getInstance(CCSparkMax::new, SwerveModuleIOHardware::new);
        AlgaeSubsystem.getInstance(CCMotorReplay::new, AlgaeIOReplay::new);
        ArmSubsystem.getInstance(CCSparkMax::new, ArmIOHardware::new);
        ClimberSubsystem.getInstance(CCMotorReplay::new, ClimberIOReplay::new);
        ElevatorSubsystem.getInstance(CCMotorReplay::new, ElevatorIOReplay::new);
        IntakeSubsystem.getInstance(CCMotorReplay::new, IntakeIOReplay::new);
        WristSubsystem.getInstance(CCSparkMax::new, WristIOHardware::new);

        Superstructure.getInstance();

        break;

      case SIM:
        System.out.println("Creating Simulated Robot.");
        SwerveDriveSubsystem.getInstance(CCSparkSim::new, SwerveModuleIOSim::new);
        AlgaeSubsystem.getInstance(CCSparkSim::new, AlgaeIOSim::new);
        ArmSubsystem.getInstance(CCSparkSim::new, ArmIOSim::new);
        ClimberSubsystem.getInstance(CCSparkMax::new, ClimberIOSim::new);
        ElevatorSubsystem.getInstance(CCSparkSim::new, ElevatorIOSim::new);
        IntakeSubsystem.getInstance(CCSparkSim::new, IntakeIOSim::new);
        WristSubsystem.getInstance(CCSparkSim::new, WristIOSim::new);

        Superstructure.getInstance();

        break;

      case REPLAY:
        System.out.println("Creating Replay Robot.");
        SwerveDriveSubsystem.getInstance(CCMotorReplay::new, SwerveModuleIOReplay::new);
        AlgaeSubsystem.getInstance(CCMotorReplay::new, AlgaeIOReplay::new);
        ArmSubsystem.getInstance(CCMotorReplay::new, ArmIOReplay::new);
        ClimberSubsystem.getInstance(CCSparkMax::new, ClimberIOReplay::new);
        ElevatorSubsystem.getInstance(CCMotorReplay::new, ElevatorIOReplay::new);
        IntakeSubsystem.getInstance(CCMotorReplay::new, IntakeIOReplay::new);
        WristSubsystem.getInstance(CCMotorReplay::new, WristIOReplay::new);

        Superstructure.getInstance();

        break;
    }

    RobotState.getInstance().robotStateInit();
    RobotState.getInstance().poseInit();
    RobotState.getInstance().moduleEncodersInit();
    RobotState.getInstance().dashboardInit();

    switch (Constants.currentMode) {
      case REAL:
        SwerveDriveScheme.configure(SwerveDriveSubsystem.getInstance(), primaryController);

        CharacterizationScheme.configure(
            SwerveDriveSubsystem.getInstance(),
            AlgaeSubsystem.getInstance(),
            ArmSubsystem.getInstance(),
            ElevatorSubsystem.getInstance(),
            IntakeSubsystem.getInstance(),
            WristSubsystem.getInstance(),
            primaryController);

        break;

      case SIM:
        SwerveDriveScheme.configure(SwerveDriveSubsystem.getInstance(), primaryController);

        SimulationScheme.configure(
            IntakeSubsystem.getInstance(),
            ArmSubsystem.getInstance(),
            ElevatorSubsystem.getInstance(),
            WristSubsystem.getInstance(),
            AlgaeSubsystem.getInstance(),
            Superstructure.getInstance(),
            primaryController);
        break;

      case REPLAY:
        break;
    }
  }
}

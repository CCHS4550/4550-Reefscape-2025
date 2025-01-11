package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.CCMotorReplay;
import frc.helpers.CCSparkMax;
import frc.helpers.CCSparkSim;
import frc.maps.Constants;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Algae.AlgaeIOHardware;
import frc.robot.Subsystems.Algae.AlgaeIOReplay;
import frc.robot.Subsystems.Algae.AlgaeIOSim;
import frc.robot.Subsystems.Algae.AlgaeSubsystem;
import frc.robot.Subsystems.Arm.ArmIOHardware;
import frc.robot.Subsystems.Arm.ArmIOReplay;
import frc.robot.Subsystems.Arm.ArmIOSim;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorIOHardware;
import frc.robot.Subsystems.Elevator.ElevatorIOReplay;
import frc.robot.Subsystems.Elevator.ElevatorIOSim;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Intake.IntakeIOHardware;
import frc.robot.Subsystems.Intake.IntakeIOReplay;
import frc.robot.Subsystems.Intake.IntakeIOSim;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.Subsystems.swervedrive.SwerveModuleIOHardware;
import frc.robot.Subsystems.swervedrive.SwerveModuleIOReplay;
import frc.robot.Subsystems.swervedrive.SwerveModuleIOSim;
import frc.robot.Subsystems.Wrist.WristIOHardware;
import frc.robot.Subsystems.Wrist.WristIOReplay;
import frc.robot.Subsystems.Wrist.WristIOSim;
import frc.robot.Subsystems.Wrist.WristSubsystem;

public class RobotContainer {

  AlgaeSubsystem algae;
  ArmSubsystem arm;
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;
  SwerveDriveSubsystem swerve;
  WristSubsystem wrist;

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

        superstructure = Superstructure.getInstance();

        break;

      case SIM:
        swerve = SwerveDriveSubsystem.getInstance(CCSparkSim::new, SwerveModuleIOSim::new);
        algae = AlgaeSubsystem.getInstance(CCSparkSim::new, AlgaeIOSim::new);
        arm = ArmSubsystem.getInstance(CCSparkSim::new, ArmIOSim::new);
        elevator = ElevatorSubsystem.getInstance(CCSparkSim::new, ElevatorIOSim::new);
        intake = IntakeSubsystem.getInstance(CCSparkSim::new, IntakeIOSim::new);
        wrist = WristSubsystem.getInstance(CCSparkSim::new, WristIOSim::new);

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
  }
}

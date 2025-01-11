package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.CCMotorReplay;
import frc.helpers.CCSparkMax;
import frc.helpers.CCSparkSim;
import frc.maps.Constants;
<<<<<<< HEAD
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae.AlgaeIOHardware;
import frc.robot.subsystems.algae.AlgaeIOReplay;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmIOHardware;
import frc.robot.subsystems.arm.ArmIOReplay;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
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
=======
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
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Wrist.WristIOHardware;
import frc.robot.Subsystems.Wrist.WristIOReplay;
import frc.robot.Subsystems.Wrist.WristIOSim;
import frc.robot.Subsystems.Wrist.WristSubsystem;
import frc.robot.Subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.Subsystems.swervedrive.SwerveModuleIOHardware;
import frc.robot.Subsystems.swervedrive.SwerveModuleIOReplay;
import frc.robot.Subsystems.swervedrive.SwerveModuleIOSim;
>>>>>>> 1a7ee7eeace158b15ff97382aa70d9cea22c89bb

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

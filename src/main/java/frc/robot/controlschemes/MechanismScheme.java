package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class MechanismScheme {
  private static CommandGenericHID buttonBoard;

  public static void configure(
      AlgaeSubsystem algae,
      ArmSubsystem arm,
      ClimberSubsystem climber,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake,
      SwerveDriveSubsystem swerve,
      WristSubsystem wrist,
      Superstructure superstructure,
      int port) {

    buttonBoard = new CommandGenericHID(port);
    configureButtons(algae, arm, climber, elevator, intake, swerve, wrist, superstructure, port);
  }

  public static void configureButtons(
      AlgaeSubsystem algae,
      ArmSubsystem arm,
      ClimberSubsystem climber,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake,
      SwerveDriveSubsystem swerve,
      WristSubsystem wrist,
      Superstructure superstructure,
      int port) {

    Trigger yellow1 = buttonBoard.button(1);
    Trigger yellow2 = buttonBoard.button(2);

    Trigger white1 = buttonBoard.button(1);
    Trigger white2 = buttonBoard.button(2);

    Trigger blue1 = buttonBoard.button(1);
    Trigger blue2 = buttonBoard.button(2);

    Trigger green1 = buttonBoard.button(1);
    Trigger green2 = buttonBoard.button(2);

    Trigger red1 = buttonBoard.button(1);
    Trigger red2 = buttonBoard.button(2);

    Trigger black1 = buttonBoard.button(1);
    Trigger black2 = buttonBoard.button(2);

    yellow1.onTrue(superstructure.intakeCoralStation());
    yellow2.onTrue(superstructure.outtakeGlobal());

    /** Unmapped controls. Will probably be algae processor. */
    white1.onTrue(superstructure.setWantedSuperstateCommand(null));
    white2.onTrue(superstructure.setWantedSuperstateCommand(null));

    blue1.onTrue(superstructure.setWantedSuperstateCommand(SuperState.L1_FRONT));
    blue2.onTrue(superstructure.setWantedSuperstateCommand(SuperState.L2_FRONT));

    green1.onTrue(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK));
    green2.onTrue(superstructure.setWantedSuperstateCommand(SuperState.L3_FRONT));

    red1.onTrue(
        superstructure.setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT));
    red2.onTrue(superstructure.setWantedSuperstateCommand(SuperState.CLIMB_PREPARING));

    black1.whileTrue(climber.climberUp());
    black2.whileTrue(climber.climberDown());
  }
}

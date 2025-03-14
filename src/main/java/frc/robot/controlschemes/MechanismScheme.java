package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.superstructure.arm.ArmSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.wrist.WristSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

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

    // intake
    //     .hasCoralTrigger()
    //     .onTrue(
    //         intake
    //             .stop()
    //             .andThen(
    //                 Commands.runOnce(
    //                     () ->
    //                         BlinkinLEDController.getInstance()
    //                             .setIfNotAlready(BlinkinPattern.STROBE_GOLD))))
    //     .onFalse(
    //         Commands.runOnce(
    //             () ->
    //                 BlinkinLEDController.getInstance()
    //                     .setIfNotAlready(BlinkinPattern.RAINBOW_RAINBOW_PALETTE)));

    // intake
    //     .hasCoralTrigger()
    //     .onTrue(Commands.runOnce(() -> intake.intakeSlow(), intake))
    //     .onFalse(runOnce(() -> intake.intakeNormal(), intake));
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

    final Trigger yellowTop = buttonBoard.button(1);
    final Trigger yellowBottom = buttonBoard.button(2);

    final Trigger whiteTop = buttonBoard.button(3);
    final Trigger whiteBottom = buttonBoard.button(4);

    final Trigger blueTop = buttonBoard.button(5);
    final Trigger blueBottom = buttonBoard.button(6);

    final Trigger greenTop = buttonBoard.button(7);
    final Trigger greenBottom = buttonBoard.button(8);

    final Trigger redTop = buttonBoard.button(9);
    final Trigger redBottom = buttonBoard.button(10);

    final Trigger blackTop = buttonBoard.button(11);
    final Trigger blackBottom = buttonBoard.button(12);

    yellowTop.onTrue(superstructure.setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT));
    yellowTop.whileTrue(superstructure.intakeCoralStation());
    yellowBottom.onTrue(
        superstructure.setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT));

    whiteTop.whileTrue(climber.climberDown());
    whiteBottom.whileTrue(climber.climberUp());

    blueTop.onTrue(superstructure.setWantedSuperstateCommand(SuperState.L1_FRONT));
    blueBottom.onTrue(superstructure.setWantedSuperstateCommand(SuperState.L2_FRONT));

    greenTop.onTrue(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK));
    greenBottom.onTrue(superstructure.setWantedSuperstateCommand(SuperState.L3_FRONT));

    redTop.onTrue(superstructure.setWantedSuperstateCommand(SuperState.KNOCK_ALGAE_TOP));
    redTop.whileTrue(intake.outtake());
    redBottom.onTrue(superstructure.setWantedSuperstateCommand(SuperState.KNOCK_ALGAE_BOTTOM));
    redBottom.whileTrue(intake.outtake());

    blackTop.whileTrue(intake.outtake());
    blackBottom.whileTrue(superstructure.setWantedSuperstateCommand(SuperState.CLIMB_PREPARING));
  }
}

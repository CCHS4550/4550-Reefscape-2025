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

    Trigger yellowTop = buttonBoard.button(1);
    Trigger yellowBottom = buttonBoard.button(2);

    Trigger whiteTop = buttonBoard.button(3);
    Trigger whiteBottom = buttonBoard.button(4);

    Trigger blueTop = buttonBoard.button(5);
    Trigger blueBottom = buttonBoard.button(6);

    Trigger greenTop = buttonBoard.button(7);
    Trigger greenBottom = buttonBoard.button(8);

    Trigger redTop = buttonBoard.button(9);
    Trigger redBottom = buttonBoard.button(10);

    Trigger blackTop = buttonBoard.button(11);
    Trigger blackBottom = buttonBoard.button(12);

    yellowTop.onTrue(superstructure.intakeCoralStation());
    yellowBottom.onTrue(superstructure.outtakeGlobal());

    /** Unmapped controls. Will probably be algae processor. */
    whiteTop.onTrue(superstructure.setWantedSuperstateCommand(SuperState.ZERO));
    // whiteBottom.onTrue(superstructure.setWantedSuperstateCommand(SuperState.ZERO));

    blueTop.onTrue(superstructure.setWantedSuperstateCommand(SuperState.L1_FRONT));
    // blueTop.onTrue(new InstantCommand(() -> System.out.println("asdfsjfkdglfh")));
    blueBottom.onTrue(superstructure.setWantedSuperstateCommand(SuperState.L2_FRONT));

    greenTop.onTrue(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK));
    greenBottom.onTrue(superstructure.setWantedSuperstateCommand(SuperState.L3_FRONT));

    redTop.onTrue(
        superstructure.setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT));
    redBottom.onTrue(superstructure.setWantedSuperstateCommand(SuperState.CLIMB_PREPARING));

    blackTop.whileTrue(climber.climberUp());
    blackBottom.whileTrue(climber.climberDown());
  }
}

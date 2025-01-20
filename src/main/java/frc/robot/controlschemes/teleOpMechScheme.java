package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class teleOpMechScheme {
  private static CommandGenericHID teleOpBoard;

  public static void configure(
      IntakeSubsystem intake,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      AlgaeSubsystem algae,
      ClimberSubsystem climber,
      int port) {
    teleOpBoard = new CommandGenericHID(port);
    configureButtons(intake, arm, elevator, wrist, algae, climber, port);
  }

  public static void configureButtons(
      IntakeSubsystem intake,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      AlgaeSubsystem algae,
      ClimberSubsystem climber,
      int port) {
    teleOpBoard.button(1).whileTrue(intake.intake()); // intake, also rebind buttons
    teleOpBoard.button(2).whileTrue(intake.outtake()); // outtake, also rebind buttons
    teleOpBoard.button(3).whileTrue(algae.wristUp()); // move algae wrist up, also rebind buttons
    teleOpBoard
        .button(4)
        .whileTrue(algae.wristDown()); // move algae wrist down, also rebind buttons
    teleOpBoard.button(5).whileTrue(algae.intake()); // intake algae, also rebind buttons
    teleOpBoard.button(6).whileTrue(algae.outtake()); // outtake algae, also rebind buttons
    teleOpBoard.button(9).whileTrue(arm.armUp()); // arm up, also rebind buttons
    teleOpBoard.button(10).whileTrue(arm.armDown()); // arm down, also rebind buttons
    teleOpBoard.button(9).whileTrue(wrist.wristUp()); // wrist up, also rebind buttons
    teleOpBoard.button(10).whileTrue(wrist.wristDown()); // wrist down, also rebind buttons
    teleOpBoard.button(11).whileTrue(climber.climberUp()); // climber up, also rebind buttons
    teleOpBoard.button(12).whileTrue(climber.climberDown()); // climber down, also rebind buttons
    teleOpBoard.button(13).whileTrue(elevator.elevatorUp()); // elevator up, also rebind buttons
    teleOpBoard.button(14).whileTrue(elevator.elevatorDown()); // elevator down, also rebind buttons
  }
}

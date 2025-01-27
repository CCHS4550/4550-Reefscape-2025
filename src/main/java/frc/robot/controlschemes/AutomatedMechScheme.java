package frc.robot.controlschemes;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeStates;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.wrist.WristSubsystem;

public class AutomatedMechScheme {
  private static CommandGenericHID autoBoard;

  public static void configure(
      IntakeSubsystem intake,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      AlgaeSubsystem algae,
      Superstructure superstructure,
      int port) {
    autoBoard = new CommandGenericHID(port);
    configureButtons(intake, arm, elevator, wrist, algae, superstructure, port);
    
  }

  public static void configureButtons(
      IntakeSubsystem intake,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      AlgaeSubsystem algae,
      Superstructure superstructure,
      int port) {
    autoBoard.button(1).onTrue(algae.setWantedStateCommand(AlgaeStates.STOW));
    autoBoard.button(2).onTrue(algae.setWantedStateCommand(AlgaeStates.INTAKE));
    autoBoard.button(3).onTrue(algae.setWantedStateCommand(AlgaeStates.PROCESSOR));
    autoBoard
        .button(4)
        .onTrue(
            superstructure.setWantedSuperstateCommand(
                WantedSuperState.WITHIN_FRAME_PERIMETER_DEFAULT));
    autoBoard
        .button(4)
        .onTrue(superstructure.setWantedSuperstateCommand(WantedSuperState.CORAL_STATION_BACK));
    autoBoard
        .button(4)
        .onTrue(superstructure.setWantedSuperstateCommand(WantedSuperState.CORAL_STATION_FRONT));
    autoBoard
        .button(4)
        .onTrue(superstructure.setWantedSuperstateCommand(WantedSuperState.L1_FRONT));
    autoBoard
        .button(4)
        .onTrue(superstructure.setWantedSuperstateCommand(WantedSuperState.L2_FRONT));
    autoBoard
        .button(4)
        .onTrue(superstructure.setWantedSuperstateCommand(WantedSuperState.L3_FRONT));
    autoBoard.button(4).onTrue(superstructure.setWantedSuperstateCommand(WantedSuperState.L4_BACK));
  }
  
}

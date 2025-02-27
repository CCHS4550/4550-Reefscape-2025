package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.helpers.BlinkinLEDController;
import frc.helpers.BlinkinLEDController.BlinkinPattern;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.superstructure.arm.ArmSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.wrist.WristSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

public class ManualMechanismScheme {
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
      int port) {

    buttonBoard = new CommandGenericHID(port);
    configureButtons(algae, arm, climber, elevator, intake, swerve, wrist, port);   
  }
  public static void configureButtons(
    AlgaeSubsystem algae,
    ArmSubsystem arm,
    ClimberSubsystem climber,
    ElevatorSubsystem elevator,
    IntakeSubsystem intake,
    SwerveDriveSubsystem swerve,
    WristSubsystem wrist,
    int port) {
        buttonBoard.button(1).whileTrue(elevator.elevatorUpCommand());
        buttonBoard.button(2).whileTrue(elevator.elevatorDownCommand());
        buttonBoard.button(3).whileTrue(arm.moveArmUpCommand());
        buttonBoard.button(4).whileTrue(arm.moveArmDownCommand());
        buttonBoard.button(5).whileTrue(wrist.wristUpCommand());
        buttonBoard.button(6).whileTrue(wrist.wristDownCommand());
    }
 }
}

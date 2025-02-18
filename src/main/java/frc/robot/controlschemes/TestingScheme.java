package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.vision.VisionIO;
import frc.robot.autonomous.*;
import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class TestingScheme {

  public static void configure(
      AlgaeSubsystem algae,
      ArmSubsystem arm,
      ClimberSubsystem climber,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake,
      SwerveDriveSubsystem swerve,
      WristSubsystem wrist,
      Superstructure superstructure,
      VisionIO vision,
      CommandXboxController controller) {

    configureButtons(
        algae, arm, climber, elevator, intake, swerve, wrist, superstructure, vision, controller);
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
      VisionIO vision,
      CommandXboxController controller) {

    /** z */
    controller.a().onTrue(new InstantCommand(() -> System.out.println("a")));
    // controller.a().whileTrue(AlignCommands.frontAlignToReefLeft(swerve, vision));

    /** x */
    controller.b().onTrue(new InstantCommand(() -> System.out.println("b")));
    controller.b().whileTrue(arm.setVoltage(1.2));

    /** c */
    controller.x().onTrue(new InstantCommand(() -> System.out.println("x")));
    controller.x().whileTrue(wrist.setVoltage(0.6));

    /** v */
    controller.y().onTrue(new InstantCommand(() -> System.out.println("y")));
    // controller.y().whileTrue(elevator.setVoltage(3));

    // controller.a().onTrue(alignToTagCommand);

    System.out.println("Configured Simulation Scheme");
  }
}

package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.*;
import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class SimulationScheme {

  public static void configure(
      IntakeSubsystem intake,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      AlgaeSubsystem algae,
      Superstructure superstructure,
      CommandXboxController controller) {

    configureButtons(intake, arm, elevator, wrist, algae, superstructure, controller);
  }

  public static void configureButtons(
      IntakeSubsystem intake,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      AlgaeSubsystem algae,
      Superstructure superstructure,
      CommandXboxController controller) {

    /** z */
    controller.a().onTrue(new InstantCommand(() -> System.out.println("a")));
    controller.a().and(AlignCommands.hasTarget()).onTrue(AlignCommands.frontAlignToReefLeft());

    /** x */
    controller.b().onTrue(new InstantCommand(() -> System.out.println("b")));

    /** c */
    controller.x().onTrue(new InstantCommand(() -> System.out.println("x")));

    /** v */
    controller.y().onTrue(new InstantCommand(() -> System.out.println("y")));

    // controller.a().onTrue(alignToTagCommand);

    System.out.println("Configured Simulation Scheme");
  }
}

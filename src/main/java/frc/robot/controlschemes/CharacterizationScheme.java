// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

/** Add your docs here. */
public class CharacterizationScheme {

  public static void configure(
      SwerveDriveSubsystem swerve,
      AlgaeSubsystem algae,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake,
      WristSubsystem wrist,
      CommandXboxController controller) {

    configureButtons(swerve, algae, arm, elevator, intake, wrist, controller);
  }

  public static void configureButtons(
      SwerveDriveSubsystem swerve,
      AlgaeSubsystem algae,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake,
      WristSubsystem wrist,
      CommandXboxController controller) {

    // controller.a().onTrue(algae.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // controller.b().onTrue(algae.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // controller.x().onTrue(algae.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // controller.y().onTrue(algae.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // controller.a().onTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // controller.b().onTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // controller.x().onTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // controller.y().onTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // controller.a().onTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // controller.b().onTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // controller.x().onTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // controller.y().onTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // controller.a().onTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // controller.b().onTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // controller.x().onTrue(intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // controller.y().onTrue(intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // controller.a().onTrue(wrist.runCharacterization());
    controller.a().onTrue(arm.setWantedStateCommand(ArmState.ZERO));

    // controller.a().onTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // controller.b().onTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // controller.x().onTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // controller.y().onTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}

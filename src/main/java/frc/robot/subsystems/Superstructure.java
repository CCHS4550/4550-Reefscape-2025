// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.superstructure.arm.ArmSubsystem;
import frc.robot.subsystems.superstructure.arm.ArmSubsystem.ArmState;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.superstructure.wrist.WristSubsystem;
import frc.robot.subsystems.superstructure.wrist.WristSubsystem.WristState;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  AlgaeSubsystem algae;
  ArmSubsystem arm;
  ClimberSubsystem climber;
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;
  SwerveDriveSubsystem swerve;
  WristSubsystem wrist;

  /** Creates a new Superstructure. */
  public Superstructure(
      AlgaeSubsystem algae,
      ArmSubsystem arm,
      ClimberSubsystem climber,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake,
      SwerveDriveSubsystem swerve,
      WristSubsystem wrist) {
    this.algae = algae;
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;
    this.swerve = swerve;
    this.wrist = wrist;
  }

  /**
   * Taken a whollle lot of inspiration from 2910 Jack-in-the-Bot's 2024 code and 1678 Citrus
   * Circuit's 2023 Code.
   */
  public enum SuperState {
    /** Idle pose, within frame perimeter. (Robot Starting Pose) */
    WITHIN_FRAME_PERIMETER_DEFAULT,
    /** Position to recieve from Coral Station (from the back) */
    CORAL_STATION_BACK,
    /** Position to recieve from Coral Station (from the front) */
    CORAL_STATION_FRONT,
    /** Position to score L1 */
    L1_FRONT,
    /** Position to score L2 */
    L2_FRONT,
    /** Position to score L3 */
    L3_FRONT,
    /** Position to Score L4 */
    L4_BACK,

    L4_INTERMEDIATE,
    /** In position to climb */
    CLIMB_PREPARING,

    KNOCK_ALGAE_BOTTOM,
    KNOCK_ALGAE_TOP,

    ZERO,
    /** FOR TESTING ONLY */
    TEST
    /** Climber actively working */
    // CLIMBING,
    /** Do anything to help robot get off the ground. */
    // CLIMB_ASSIST
  }

  SuperState wantedSuperState = SuperState.WITHIN_FRAME_PERIMETER_DEFAULT;
  public SuperState currentSuperState = SuperState.WITHIN_FRAME_PERIMETER_DEFAULT;
  SuperState previousSuperState = SuperState.WITHIN_FRAME_PERIMETER_DEFAULT;

  public static boolean outtakeReverse = false;

  public void applyStates() {
    switch (currentSuperState) {
      case ZERO:
        arm.setWantedStateCommand(ArmState.ZERO).schedule();

        wrist.setWantedStateCommand(WristState.ZERO).schedule();

        elevator.setWantedStateCommand(ElevatorState.ZERO).schedule();
        break;

      case WITHIN_FRAME_PERIMETER_DEFAULT:
        arm.setWantedStateCommand(ArmState.DEFAULT_WITHINFRAME).schedule();

        wrist.setWantedStateCommand(WristState.DEFAULT_WITHINFRAME).schedule();

        new WaitCommand(1).andThen(elevator.setWantedStateCommand(ElevatorState.ZERO)).schedule();
        break;

      case CORAL_STATION_BACK:
        arm.setWantedStateCommand(ArmState.CORAL_STATION_BACK).schedule();

        wrist.setWantedStateCommand(WristState.CORAL_STATION_BACK).schedule();

        new WaitCommand(1).andThen(elevator.setWantedStateCommand(ElevatorState.ZERO)).schedule();
        break;

      case CORAL_STATION_FRONT:
        arm.setWantedStateCommand(ArmState.CORAL_STATION_FRONT).schedule();

        wrist.setWantedStateCommand(WristState.CORAL_STATION_FRONT).schedule();

        elevator.setWantedStateCommand(ElevatorState.ZERO).schedule();
        outtakeReverse = false;
        break;

      case L1_FRONT:
        arm.setWantedStateCommand(ArmState.L1_FRONT).schedule();

        wrist.setWantedStateCommand(WristState.L1_FRONT).schedule();

        elevator.setWantedStateCommand(ElevatorState.L1_FRONT).schedule();
        outtakeReverse = false;
        break;

      case L2_FRONT:
        arm.setWantedStateCommand(ArmState.L2_FRONT).schedule();

        wrist.setWantedStateCommand(WristState.L2_FRONT).schedule();

        elevator.setWantedStateCommand(ElevatorState.L2_FRONT).schedule();
        outtakeReverse = false;
        break;

      case L3_FRONT:
        arm.setWantedStateCommand(ArmState.L3_FRONT).schedule();

        wrist.setWantedStateCommand(WristState.L3_FRONT).schedule();

        elevator.setWantedStateCommand(ElevatorState.L3_FRONT).schedule();
        outtakeReverse = false;
        break;

      case L4_BACK:
        arm.setWantedStateCommand(ArmState.L4_BACK).schedule();

        wrist.setWantedStateCommand(WristState.L4_BACK).schedule();

        new WaitCommand(0)
            .andThen(elevator.setWantedStateCommand(ElevatorState.L4_BACK))
            .schedule();
        outtakeReverse = true;
        break;
      case L4_INTERMEDIATE:
        arm.setWantedStateCommand(ArmState.L3_FRONT).schedule();

        wrist.setWantedStateCommand(WristState.L1_FRONT).schedule();

        elevator.setWantedStateCommand(ElevatorState.ZERO).schedule();

        new WaitCommand(0).andThen(setWantedSuperstateCommand(wantedSuperState)).schedule();
        wantedSuperState = SuperState.L4_INTERMEDIATE;
        outtakeReverse = true;

        break;

      case KNOCK_ALGAE_BOTTOM:
        arm.setWantedStateCommand(ArmState.L3_FRONT).schedule();
        new WaitCommand(2)
          .andThen(arm.setWantedStateCommand(ArmState.L4_BACK))
          .schedule();

        wrist.setWantedStateCommand(WristState.L4_BACK).schedule();
        new WaitCommand(2)
          .andThen(wrist.setWantedStateCommand(WristState.L4_BACK))
          .schedule();

        elevator.setWantedStateCommand(ElevatorState.L1_FRONT)
          .schedule();

      case KNOCK_ALGAE_TOP:
          arm.setWantedStateCommand(ArmState.L3_FRONT).schedule();
          new WaitCommand(2)
          .andThen(arm.setWantedStateCommand(ArmState.L4_BACK))
          .schedule();
  
          wrist.setWantedStateCommand(WristState.L4_BACK).schedule();
          new WaitCommand(2)
          .andThen(wrist.setWantedStateCommand(WristState.L4_BACK))
          .schedule();
  
          elevator.setWantedStateCommand(ElevatorState.L3_FRONT)
            .schedule();
  

      case CLIMB_PREPARING:
        //   arm.setWantedStateCommand(ArmState.CLIMB_PREPARING).schedule();

        //   wrist.setWantedStateCommand(WristState.CLIMB_PREPARING).schedule();

        //   elevator.setWantedStateCommand(ElevatorState.CLIMB_PREPARING).schedule();

        arm.setWantedStateCommand(ArmState.L4_BACK).schedule();

        wrist.setWantedStateCommand(WristState.L4_BACK).schedule();

        elevator.setWantedStateCommand(ElevatorState.ZERO).schedule();
        outtakeReverse = false;
        break;
      default:
        break;
    }
  }

  private SuperState handleStateTransitions() {
    previousSuperState = currentSuperState;

    switch (wantedSuperState) {
      case WITHIN_FRAME_PERIMETER_DEFAULT:
        return currentSuperState = SuperState.WITHIN_FRAME_PERIMETER_DEFAULT;

      case ZERO:
        if (currentSuperState == SuperState.L4_BACK)
          return currentSuperState = SuperState.L4_INTERMEDIATE;
        return currentSuperState = SuperState.ZERO;

      case CORAL_STATION_BACK:
        if (currentSuperState == SuperState.L4_BACK)
          return currentSuperState = SuperState.L4_INTERMEDIATE;
        return currentSuperState = SuperState.CORAL_STATION_BACK;

      case CORAL_STATION_FRONT:
        if (currentSuperState == SuperState.L4_BACK)
          return currentSuperState = SuperState.L4_INTERMEDIATE;
        return currentSuperState = SuperState.CORAL_STATION_FRONT;

      case L1_FRONT:
        if (currentSuperState == SuperState.L4_BACK)
          return currentSuperState = SuperState.L4_INTERMEDIATE;
        return currentSuperState = SuperState.L1_FRONT;

      case L2_FRONT:
        if (currentSuperState == SuperState.L4_BACK)
          return currentSuperState = SuperState.L4_INTERMEDIATE;
        return currentSuperState = SuperState.L2_FRONT;

      case L3_FRONT:
        if (currentSuperState == SuperState.L4_BACK)
          return currentSuperState = SuperState.L4_INTERMEDIATE;
        return currentSuperState = SuperState.L3_FRONT;

      case L4_BACK:
        if (currentSuperState == SuperState.WITHIN_FRAME_PERIMETER_DEFAULT
            || currentSuperState == SuperState.L1_FRONT
            || currentSuperState == SuperState.ZERO) {
          return currentSuperState = SuperState.L4_INTERMEDIATE;
        }
        return currentSuperState = SuperState.L4_BACK;

      case CLIMB_PREPARING:
        if (currentSuperState == SuperState.L4_BACK)
          return currentSuperState = SuperState.L4_INTERMEDIATE;
        return currentSuperState = SuperState.CLIMB_PREPARING;

      case KNOCK_ALGAE_BOTTOM: 
        if (currentSuperState == SuperState.L4_BACK)
          return currentSuperState = SuperState.L4_INTERMEDIATE;
        return currentSuperState = SuperState.KNOCK_ALGAE_BOTTOM;
      case KNOCK_ALGAE_TOP: 
        if (currentSuperState == SuperState.L4_BACK)
          return currentSuperState = SuperState.L4_INTERMEDIATE;
        return currentSuperState = SuperState.KNOCK_ALGAE_TOP;


      default:
        return currentSuperState = SuperState.WITHIN_FRAME_PERIMETER_DEFAULT;
    }
  }

  public void setWantedSuperstate(SuperState wantedSuperState) {
    this.wantedSuperState = wantedSuperState;
  }

  public Command setWantedSuperstateCommand(SuperState wantedSuperState) {
    return new InstantCommand(() -> setWantedSuperstate(wantedSuperState));
  }

  // private Command setCurrentSuperstateCommand(SuperState currentSuperState) {
  //   return new InstantCommand(() -> this.currentSuperState = currentSuperState);
  // }

  public SuperState getWantedSuperstate() {
    return wantedSuperState;
  }

  public Command intakeCoralStation() {
    return intake.intakeCoralStation();
  }

  public Command outtakeGlobal() {
    return intake.outtake();
  }

  public static boolean shouldReverse() {
    return outtakeReverse;
  }

  public Trigger isL4() {
    return new Trigger(() -> currentSuperState == SuperState.L4_BACK);
  }

  public Trigger isNotL4() {
    return new Trigger(() -> currentSuperState != SuperState.L4_BACK);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.recordOutput("Subsystem/Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Subsystem/Superstructure/CurrentSuperState", currentSuperState);
    Logger.recordOutput("Subsystem/Superstructure/PreviousSuperState", previousSuperState);

    Logger.recordOutput("reverse Outtake", outtakeReverse);
    if (wantedSuperState != currentSuperState) {
      currentSuperState = handleStateTransitions();
      applyStates();
    }
  }
}

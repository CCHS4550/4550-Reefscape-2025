// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem.WristState;
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
    /** In position to climb */
    CLIMB_PREPARING,

    ZERO,
    /** FOR TESTING ONLY */
    TEST
    /** Climber actively working */
    // CLIMBING,
    /** Do anything to help robot get off the ground. */
    // CLIMB_ASSIST
  }

  SuperState wantedSuperState = SuperState.WITHIN_FRAME_PERIMETER_DEFAULT;
  SuperState currentSuperState = SuperState.WITHIN_FRAME_PERIMETER_DEFAULT;
  SuperState previousSuperState = SuperState.WITHIN_FRAME_PERIMETER_DEFAULT;

  public void applyStates() {
    switch (currentSuperState) {
      case ZERO:
        arm.setWantedState(ArmState.ZERO);
        // elevator.setWantedState(ElevatorState.CORAL_STATION_BACK);
        wrist.setWantedState(WristState.ZERO);
        break;

      case WITHIN_FRAME_PERIMETER_DEFAULT:
        // arm.setWantedState(ArmState.DEFAULT_WITHINFRAME);
        // elevator.setWantedState(ElevatorState.DEFAULT_WITHINFRAME);
        // wrist.setWantedState(WristState.DEFAULT_WITHINFRAME);
        break;

      case CORAL_STATION_BACK:
        arm.setWantedState(ArmState.CORAL_STATION_BACK);
        // elevator.setWantedState(ElevatorState.CORAL_STATION_BACK);
        wrist.setWantedState(WristState.CORAL_STATION_BACK);
        break;

      case CORAL_STATION_FRONT:
        arm.setWantedState(ArmState.CORAL_STATION_FRONT);
        // elevator.setWantedState(ElevatorState.CORAL_STATION_FRONT);
        wrist.setWantedState(WristState.CORAL_STATION_FRONT);
        break;

      case L1_FRONT:
        arm.setWantedState(ArmState.L1_FRONT);
        // elevator.setWantedState(ElevatorState.L1_FRONT);
        wrist.setWantedState(WristState.L1_FRONT);
        break;

      case L2_FRONT:
        arm.setWantedState(ArmState.L2L3_FRONT);
        // elevator.setWantedState(ElevatorState.L2_FRONT);
        wrist.setWantedState(WristState.L2L3_FRONT);
        break;

      case L3_FRONT:
        arm.setWantedState(ArmState.L2L3_FRONT);
        // elevator.setWantedState(ElevatorState.L3_FRONT);
        wrist.setWantedState(WristState.L2L3_FRONT);
        break;

      case L4_BACK:
        arm.setWantedState(ArmState.L4_BACK);
        // elevator.setWantedState(ElevatorState.L4_BACK);
        wrist.setWantedState(WristState.L4_BACK);
        break;
      case CLIMB_PREPARING:
        arm.setWantedState(ArmState.CLIMB_PREPARING);
        // elevator.setWantedState(ElevatorState.CLIMB_PREPARING);
        wrist.setWantedState(WristState.CLIMB_PREPARING);
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
        return currentSuperState = SuperState.ZERO;

      case CORAL_STATION_BACK:
        return currentSuperState = SuperState.CORAL_STATION_BACK;

      case CORAL_STATION_FRONT:
        return currentSuperState = SuperState.CORAL_STATION_FRONT;

      case L1_FRONT:
        return currentSuperState = SuperState.L1_FRONT;

      case L2_FRONT:
        return currentSuperState = SuperState.L2_FRONT;

      case L3_FRONT:
        return currentSuperState = SuperState.L3_FRONT;

      case L4_BACK:
        return currentSuperState = SuperState.L4_BACK;

      case CLIMB_PREPARING:
        return currentSuperState = SuperState.CLIMB_PREPARING;

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

  public SuperState getWantedSuperstate() {
    return wantedSuperState;
  }

  public Command intakeCoralStation() {
    return intake.intakeCoralStation();
  }

  public Command outtakeGlobal() {
    if (elevator.currentState == ElevatorState.L4_BACK) return intake.outtakeBack();
    else return intake.outtakeFront();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.recordOutput("Subsystem/Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Subsystem/Superstructure/CurrentSuperState", currentSuperState);
    if (wantedSuperState != currentSuperState) currentSuperState = handleStateTransitions();
    applyStates();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem;

public class Superstructure extends SubsystemBase {

  /** Implementation of Singleton Pattern */
  public static Superstructure mInstance;

  public static Superstructure getInstance() {
    if (mInstance == null) {
      mInstance = new Superstructure();
    }
    return mInstance;
  }

  AlgaeSubsystem algae;
  ArmSubsystem arm;
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;
  SwerveDriveSubsystem swerve;
  WristSubsystem wrist;

  /** Creates a new Superstructure. */
  private Superstructure() {
    algae = AlgaeSubsystem.getInstance();
    arm = ArmSubsystem.getInstance();
    elevator = ElevatorSubsystem.getInstance();
    intake = IntakeSubsystem.getInstance();
    swerve = SwerveDriveSubsystem.getInstance();
    wrist = WristSubsystem.getInstance();
  }

  /**
   * Taken a whollle lot of inspiration from 2910 Jack-in-the-Bot's 2024 code and 1678 Citrus
   * Circuit's 2023 Code.
   */
  public enum WantedSuperState {
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
    /** Climber actively working */
    CLIMBING,
    /** Do anything to help robot get off the ground. */
    CLIMB_ASSIST
  }

  public enum CurrentSuperState {
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
    /** Climber actively working */
    CLIMBING,
    /** Do anything to help robot get off the ground. */
    CLIMB_ASSIST
  }

  WantedSuperState wantedSuperState = WantedSuperState.WITHIN_FRAME_PERIMETER_DEFAULT;
  CurrentSuperState currentSuperState = CurrentSuperState.WITHIN_FRAME_PERIMETER_DEFAULT;
  CurrentSuperState previousSuperState = CurrentSuperState.WITHIN_FRAME_PERIMETER_DEFAULT;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    currentSuperState = handleStateTransitions();
    applyStates();
  }

  private CurrentSuperState handleStateTransitions() {
    previousSuperState = currentSuperState;
    switch (wantedSuperState) {
      case WITHIN_FRAME_PERIMETER_DEFAULT:
        currentSuperState = CurrentSuperState.WITHIN_FRAME_PERIMETER_DEFAULT;
        break;

      case CORAL_STATION_BACK:
        currentSuperState = CurrentSuperState.CORAL_STATION_BACK;
        break;

      case CORAL_STATION_FRONT:
        currentSuperState = CurrentSuperState.CORAL_STATION_FRONT;
        break;

      case L1_FRONT:
        currentSuperState = CurrentSuperState.L1_FRONT;
        break;

      case L2_FRONT:
        currentSuperState = CurrentSuperState.L2_FRONT;
        break;

      case L3_FRONT:
        currentSuperState = CurrentSuperState.L3_FRONT;
        break;

      case L4_BACK:
        currentSuperState = CurrentSuperState.L4_BACK;
        break;

        /**
         * On Second thought, climbing should not be part of the state machine. I believe it should
         * be manual.
         */
        // case CLIMB_PREPARING:
        //   currentSuperState = CurrentSuperState.CLIMB_PREPARING;
        //   break;
        // case CLIMBING:
        //   currentSuperState = CurrentSuperState.CLIMBING;
        //   break;
        // case CLIMB_ASSIST:
        //   currentSuperState = CurrentSuperState.CLIMB_ASSIST;
        //   break;

      default:
        currentSuperState = currentSuperState.WITHIN_FRAME_PERIMETER_DEFAULT;
    }
    return currentSuperState;
  }

  public void applyStates() {
    switch (currentSuperState) {
      case WITHIN_FRAME_PERIMETER_DEFAULT:
        break;

      case CORAL_STATION_BACK:
        break;

      case CORAL_STATION_FRONT:
        break;

      case L1_FRONT:
        break;

      case L2_FRONT:
        break;

      case L3_FRONT:
        break;

      case L4_BACK:
        break;
    }
  }
}

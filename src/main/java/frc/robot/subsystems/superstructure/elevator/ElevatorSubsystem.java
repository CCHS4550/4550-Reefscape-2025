// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.util.maps.Constants;
import frc.util.motorcontroller.CCMotorController;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

  public enum ElevatorState {
    // Placeholder Values
    ZERO(0),
    // DEFAULT_WITHINFRAME(0),
    L1_FRONT(0.2),
    L2_FRONT(0),
    L3_FRONT(0.38),
    L4_INTERMEDIATE(0.25),
    L4_BACK(.525),
    // A1(0),
    // A2(0),
    KNOCK_ALGAE_TOP(.35),
    KNOCK_ALGAE_BOTTOM(0),
    MANUAL(0),
    CLIMB_PREPARING(0);

    private final double heightMeters;

    ElevatorState(double heightMeters) {
      this.heightMeters = heightMeters;
    }

    public double getHeight() {
      return heightMeters;
    }
  }

  public ElevatorState previousState = ElevatorState.ZERO;
  public ElevatorState currentState = ElevatorState.ZERO;
  public ElevatorState wantedState = ElevatorState.ZERO;

  private final ElevatorIO elevatorIO;

  private final SysIdRoutine sysIdRoutine;

  public final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

  Trigger hallEffectTripped = new Trigger(() -> elevatorInputs.hallEffectTripped);

  /** Creates a new WristSubsystem. */
  public ElevatorSubsystem(
      CCMotorController.MotorFactory motorFactory, ElevatorIO.IOFactory ioFactory) {

    this.elevatorIO =
        ioFactory.create(
            motorFactory.create(
                "elevatorBottom",
                "bottom",
                Constants.MotorConstants.ELEVATOR[0],
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.ELEVATOR_REVERSE[0],
                1,
                1),
            motorFactory.create(
                "elevatorTop",
                "top",
                Constants.MotorConstants.ELEVATOR[1],
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.ELEVATOR_REVERSE[1],
                1,
                1));

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1),
                Volts.of(2),
                Seconds.of(2),
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.elevatorIO.setVoltage(voltage),
                null, // No log consumer, since data is recorded by URCL
                this));

    hallEffectTripped.onTrue(resetEncoderCommand());
  }

  private void applyStates() {
    switch (currentState) {
        // case DEFAULT_WITHINFRAME:
        //   elevatorIO.holdAtState(ElevatorState.DEFAULT_WITHINFRAME);
        //   break;

      case L1_FRONT:
        elevatorIO.holdAtState(ElevatorState.L1_FRONT);
        break;
      case L4_INTERMEDIATE:
        elevatorIO.holdAtStateWithVelocity(ElevatorState.L4_INTERMEDIATE, 5);

      case L2_FRONT:
        elevatorIO.holdAtState(ElevatorState.L2_FRONT);
        break;
      case L3_FRONT:
        elevatorIO.holdAtState(ElevatorState.L3_FRONT);
        break;

      case L4_BACK:
        elevatorIO.holdAtState(ElevatorState.L4_BACK);
        break;

      case KNOCK_ALGAE_TOP:
        elevatorIO.holdAtState(ElevatorState.KNOCK_ALGAE_TOP);
        break;

      case KNOCK_ALGAE_BOTTOM:
        elevatorIO.holdAtState(ElevatorState.KNOCK_ALGAE_BOTTOM);
        break;
      case MANUAL:
        break;
      case CLIMB_PREPARING:
        elevatorIO.holdAtState(ElevatorState.CLIMB_PREPARING);
        break;

      default:
        elevatorIO.holdAtState(ElevatorState.ZERO);
        break;
    }
  }

  private ElevatorState handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
        // case DEFAULT_WITHINFRAME:
        //   return ElevatorState.DEFAULT_WITHINFRAME;

      case L1_FRONT:
        return ElevatorState.L1_FRONT;
      case L4_INTERMEDIATE:
        return ElevatorState.L4_INTERMEDIATE;

      case L2_FRONT:
        return ElevatorState.L2_FRONT;
      case L3_FRONT:
        return ElevatorState.L3_FRONT;

      case L4_BACK:
        return ElevatorState.L4_BACK;

      case KNOCK_ALGAE_TOP:
        return ElevatorState.KNOCK_ALGAE_TOP;

      case KNOCK_ALGAE_BOTTOM:
        return ElevatorState.KNOCK_ALGAE_BOTTOM;
      case MANUAL:
        return ElevatorState.MANUAL;
      default:
        return ElevatorState.ZERO;
    }
  }

  public void setWantedState(ElevatorState wantedState) {
    this.wantedState = wantedState;
  }

  public Command setWantedStateCommand(ElevatorState wantedSuperState) {
    return new InstantCommand(() -> setWantedState(wantedSuperState));
  }

  public ElevatorState getElevatorState() {
    return currentState;
  }

  public Command elevatorUpCommand() {
    return Commands.startEnd(
        () -> elevatorIO.setVoltage(Volts.of(3)), () -> elevatorIO.setVoltage(Volts.of(0)));
  }

  public Command elevatorDownCommand() {
    return Commands.startEnd(
        () -> elevatorIO.setVoltage(Volts.of(-3)), () -> elevatorIO.setVoltage(Volts.of(0)));
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Subsystem/Elevator/CurrentState", currentState.name());
    Logger.recordOutput("Subsystem/Elevator/WantedState", wantedState.name());

    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs("Subsystem/Elevator", elevatorInputs);

    if (RobotState.getInstance().allowSubsystemMovement.getAsBoolean()
        && RobotState.getInstance().moveElevator.getAsBoolean()) {
      if (wantedState != currentState) {
        currentState = handleStateTransitions();
      }
      applyStates();
    }

    // This method will be called once per scheduler run
  }

  public Command resetEncoderCommand() {
    return new InstantCommand(() -> elevatorIO.resetEncoders(), this);
  }

  public void resetPID() {
    elevatorIO.resetPID();
  }

  public Command elevatorUp() {
    return this.startEnd(
        () -> {
          elevatorIO.setVoltage(Volts.of(5.0));
        },
        () -> {
          elevatorIO.setVoltage(Volts.of(0.0));
        });
  }

  public Command elevatorDown() {
    return this.startEnd(
        () -> {
          elevatorIO.setVoltage(Volts.of(-5.0));
        },
        () -> {
          elevatorIO.setVoltage(Volts.of(0.0));
        });
  }

  public Command testVoltageCommand(double volts) {
    return Commands.startEnd(
        () -> elevatorIO.setVoltage(Volts.of(volts)), () -> elevatorIO.setVoltage(Volts.of(0)));
  }

  /* SysID Factory Methods */

  // /**
  //  * Used only in characterizing. Don't touch this.
  // //  *
  //  * @param direction
  //  * @return the quasistatic characterization test
  //  */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  // /**
  //  * Used only in characterizing. Don't touch this.
  //  *
  //  * @param direction
  //  * @return the dynamic characterization test
  //  */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}

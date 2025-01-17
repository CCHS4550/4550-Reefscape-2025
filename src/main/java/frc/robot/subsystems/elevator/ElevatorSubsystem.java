// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  public enum ElevatorState {
    // Placeholder Values
    DEFAULT_WITHINFRAME(Constants.ElevatorConstants.elevatorPositions[0]),
    L1_FRONT(Constants.ElevatorConstants.elevatorPositions[1]),
    L2_FRONT(Constants.ElevatorConstants.elevatorPositions[2]),
    L3_FRONT(Constants.ElevatorConstants.elevatorPositions[3]),
    L4_BACK(Constants.ElevatorConstants.elevatorPositions[4]),
    A1(Constants.ElevatorConstants.elevatorPositions[5]),
    A2(Constants.ElevatorConstants.elevatorPositions[6]),
    CORAL_STATION_FRONT(Constants.ElevatorConstants.elevatorPositions[7]),
    CORAL_STATION_BACK(Constants.ElevatorConstants.elevatorPositions[8]);

    private final double heightMeters;

    ElevatorState(double heightMeters) {
      this.heightMeters = heightMeters;
    }

    public double getHeight() {
      return heightMeters;
    }
  }

  public ElevatorState previousState = ElevatorState.DEFAULT_WITHINFRAME;
  public ElevatorState currentState = ElevatorState.DEFAULT_WITHINFRAME;
  public ElevatorState wantedState = ElevatorState.DEFAULT_WITHINFRAME;

  /** Implementation of Singleton Pattern */
  public static ElevatorSubsystem mInstance;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCSparkMax::new;
  private static ElevatorIO.IOFactory defaultIoFactory = ElevatorIOHardware::new;

  CCMotorController.MotorFactory motorFactory;
  ElevatorIO.IOFactory ioFactory;

  ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

  public static ElevatorSubsystem getInstance(
      CCMotorController.MotorFactory motorFactory, ElevatorIO.IOFactory ioFactory) {
    if (mInstance == null) {
      mInstance = new ElevatorSubsystem(motorFactory, ioFactory);
    }
    return mInstance;
  }

  public static ElevatorSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ElevatorSubsystem(defaultMotorFactory, defaultIoFactory);
    }
    return mInstance;
  }

  /** Creates a new WristSubsystem. */
  private ElevatorSubsystem(
      CCMotorController.MotorFactory motorFactory, ElevatorIO.IOFactory ioFactory) {
    this.motorFactory = motorFactory;
    this.ioFactory = ioFactory;
  }

  private final ElevatorIO io =
      ioFactory.create(
          motorFactory.create(
              "elevatorMotor1",
              "elevator1",
              Constants.MotorConstants.ELEVATOR[0],
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.ELEVATOR_REVERSE[0],
              Constants.ElevatorConstants.HEIGHT_METERS_PER_ELEVATOR_MOTOR_ROTATIONS,
              Constants.ElevatorConstants.ELEVATOR_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
          motorFactory.create(
              "elevatorMotor2",
              "elevator2",
              Constants.MotorConstants.ELEVATOR[1],
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.ELEVATOR_REVERSE[1],
              Constants.ElevatorConstants.HEIGHT_METERS_PER_ELEVATOR_MOTOR_ROTATIONS,
              Constants.ElevatorConstants.ELEVATOR_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR));

  private void applyStates() {
    switch (currentState) {
      case DEFAULT_WITHINFRAME:
        io.holdAtState(ElevatorState.DEFAULT_WITHINFRAME);

      case L1_FRONT:
        io.holdAtState(ElevatorState.L1_FRONT);

      case L2_FRONT:
        io.holdAtState(ElevatorState.L2_FRONT);
      case L3_FRONT:
        io.holdAtState(ElevatorState.L3_FRONT);

      case L4_BACK:
        io.holdAtState(ElevatorState.L4_BACK);

      case CORAL_STATION_FRONT:
        io.holdAtState(ElevatorState.CORAL_STATION_FRONT);

      case CORAL_STATION_BACK:
        io.holdAtState(ElevatorState.CORAL_STATION_BACK);

      default:
        io.holdAtState(ElevatorState.DEFAULT_WITHINFRAME);
    }
  }

  private ElevatorState handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case DEFAULT_WITHINFRAME:
        return ElevatorState.DEFAULT_WITHINFRAME;

      case L1_FRONT:
        return ElevatorState.L1_FRONT;

      case L2_FRONT:
        return ElevatorState.L2_FRONT;
      case L3_FRONT:
        return ElevatorState.L3_FRONT;

      case L4_BACK:
        return ElevatorState.L4_BACK;

      case CORAL_STATION_FRONT:
        return ElevatorState.CORAL_STATION_FRONT;

      case CORAL_STATION_BACK:
        return ElevatorState.CORAL_STATION_BACK;

      default:
        return ElevatorState.DEFAULT_WITHINFRAME;
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

  @Override
  public void periodic() {
    io.updateInputs(elevatorInputs);
    currentState = handleStateTransitions();
    applyStates();

    // This method will be called once per scheduler run
  }

  public Command elevatorUp() {
    return this.startEnd(
        () -> {
          io.setVoltage(Volts.of(5.0));
        },
        () -> {
          io.setVoltage(Volts.of(0.0));
        });
  }

  public Command elevatorDown() {
    return this.startEnd(
        () -> {
          io.setVoltage(Volts.of(-5.0));
        },
        () -> {
          io.setVoltage(Volts.of(0.0));
        });
  }
}

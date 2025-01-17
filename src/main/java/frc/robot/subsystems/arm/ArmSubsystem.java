// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class ArmSubsystem extends SubsystemBase {

  public enum ArmState {
    // Placeholder Values
    DEFAULT_WITHINFRAME(Units.degreesToRadians(0)),
    L1_FRONT(Units.degreesToRadians(30)),
    L2L3_FRONT(Units.degreesToRadians(45)),
    L4_BACK(Units.degreesToRadians(110)),
    CORAL_STATION_FRONT(Units.degreesToRadians(15)),
    CORAL_STATION_BACK(Units.degreesToRadians(120));

    public final double angleRadians;

    ArmState(double angleRadians) {
      this.angleRadians = angleRadians;
    }

    public double getAngle() {
      return angleRadians;
    }
  }

  public ArmState previousState = ArmState.DEFAULT_WITHINFRAME;
  public ArmState currentState = ArmState.DEFAULT_WITHINFRAME;
  public ArmState wantedState = ArmState.DEFAULT_WITHINFRAME;

  /** Implementation of Singleton Pattern */
  public static ArmSubsystem mInstance;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCSparkMax::new;
  private static ArmIO.IOFactory defaultIoFactory = ArmIOHardware::new;

  CCMotorController.MotorFactory motorFactory;
  ArmIO.IOFactory ioFactory;

  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  public static ArmSubsystem getInstance(
      CCMotorController.MotorFactory motorFactory, ArmIO.IOFactory ioFactory) {
    if (mInstance == null) {
      mInstance = new ArmSubsystem(motorFactory, ioFactory);
    }
    return mInstance;
  }

  public static ArmSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ArmSubsystem(defaultMotorFactory, defaultIoFactory);
    }
    return mInstance;
  }

  /** Creates a new WristSubsystem. */
  private ArmSubsystem(CCMotorController.MotorFactory motorFactory, ArmIO.IOFactory ioFactory) {
    this.motorFactory = motorFactory;
    this.ioFactory = ioFactory;
  }

  private final ArmIO io =
      ioFactory.create(
          motorFactory.create(
              "armMotor",
              "arm",
              Constants.MotorConstants.ARM,
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.ARM_REVERSE,
              Constants.ArmConstants.ARM_MOTOR_ROTATIONS_TO_ARM_ROTATIONS_RADIANS,
              Constants.ArmConstants.ARM_MOTOR_RADIANS_PER_SECOND_CONVERSION_FACTOR));

  private void applyStates() {
    switch (currentState) {
      case DEFAULT_WITHINFRAME:
        io.holdAtState(ArmState.DEFAULT_WITHINFRAME);

      case L1_FRONT:
        io.holdAtState(ArmState.L1_FRONT);

      case L2L3_FRONT:
        io.holdAtState(ArmState.L2L3_FRONT);

      case L4_BACK:
        io.holdAtState(ArmState.L4_BACK);

      case CORAL_STATION_FRONT:
        io.holdAtState(ArmState.CORAL_STATION_FRONT);

      case CORAL_STATION_BACK:
        io.holdAtState(ArmState.CORAL_STATION_BACK);

      default:
        io.holdAtState(ArmState.DEFAULT_WITHINFRAME);
    }
  }

  private ArmState handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case DEFAULT_WITHINFRAME:
        return ArmState.DEFAULT_WITHINFRAME;

      case L1_FRONT:
        return ArmState.L1_FRONT;

      case L2L3_FRONT:
        return ArmState.L2L3_FRONT;

      case L4_BACK:
        return ArmState.L4_BACK;

      case CORAL_STATION_FRONT:
        return ArmState.CORAL_STATION_FRONT;

      case CORAL_STATION_BACK:
        return ArmState.CORAL_STATION_BACK;

      default:
        return ArmState.DEFAULT_WITHINFRAME;
    }
  }

  public void setWantedState(ArmState wantedState) {
    this.wantedState = wantedState;
  }

  public Command setWantedStateCommand(ArmState wantedSuperState) {
    return new InstantCommand(() -> setWantedState(wantedSuperState));
  }

  public ArmState getWantedState() {
    return wantedState;
  }

  @Override
  public void periodic() {
    io.updateInputs(armInputs);
    currentState = handleStateTransitions();
    applyStates();

    // This method will be called once per scheduler run
  }

  public Command armUp() {
    return this.startEnd(
        () -> {
          io.setVoltage(Volts.of(5.0));
        },
        () -> {
          io.setVoltage(Volts.of(0.0));
        });
  }

  public Command armDown() {
    return this.startEnd(
        () -> {
          io.setVoltage(Volts.of(-5.0));
        },
        () -> {
          io.setVoltage(Volts.of(0.0));
        });
  }
}

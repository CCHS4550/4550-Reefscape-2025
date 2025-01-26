// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCMotorReplay;
import frc.helpers.Elastic;
import frc.helpers.Elastic.Notification.NotificationLevel;
import frc.helpers.OI;
import frc.maps.Constants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public double lastDetectedBeamBreakTimestamp;

  public enum IntakeState {
    IDLE,
    INTAKING_FRONT,
    INTAKING_BACK,
    OUTTAKING_FRONT,
    OUTTAKING_BACK,
    NO_CORAL,
    HAS_CORAL;
  }

  public IntakeState previousState = IntakeState.IDLE;
  public IntakeState currentState = IntakeState.IDLE;
  public IntakeState wantedState = IntakeState.IDLE;

  /** Implementation of Singleton Pattern */
  public static IntakeSubsystem mInstance;

  private final IntakeIO io;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCMotorReplay::new;
  private static IntakeIO.IOFactory defaultIoFactory = IntakeIOReplay::new;

  CCMotorController.MotorFactory motorFactory;
  IntakeIO.IOFactory ioFactory;

  IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  public static IntakeSubsystem getInstance(
      CCMotorController.MotorFactory motorFactory, IntakeIO.IOFactory ioFactory) {
    if (mInstance == null) {
      mInstance = new IntakeSubsystem(motorFactory, ioFactory);
    }
    return mInstance;
  }

  public static IntakeSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new IntakeSubsystem(defaultMotorFactory, defaultIoFactory);
      System.out.println("CREATING DEFAULT INTAKE");
    }
    return mInstance;
  }

  /** Creates a new WristSubsystem. */
  private IntakeSubsystem(
      CCMotorController.MotorFactory motorFactory, IntakeIO.IOFactory ioFactory) {
    this.motorFactory = motorFactory;
    this.ioFactory = ioFactory;

    this.io =
        ioFactory.create(
            motorFactory.create(
                "intakeMotor1",
                "intake1",
                Constants.MotorConstants.INTAKE[0],
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.INTAKE_REVERSE[0],
                1,
                1),
            motorFactory.create(
                "intakeMotor2",
                "intake2",
                Constants.MotorConstants.INTAKE[1],
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.INTAKE_REVERSE[1],
                1,
                1));
  }

  private void applyStates() {
    switch (currentState) {
      case IDLE:
        io.setInnerVoltage(Volts.of(0));
        io.setOuterVoltage(Volts.of(0));

      case INTAKING_FRONT:
        io.setInnerVoltage(Volts.of(5));
        io.setOuterVoltage(Volts.of(5));

      case INTAKING_BACK:
        io.setInnerVoltage(Volts.of(5));
        io.setOuterVoltage(Volts.of(5));

        if (true) {
          wantedState = IntakeState.HAS_CORAL;
        }
      case OUTTAKING_FRONT:
        io.setInnerVoltage(Volts.of(5));
        io.setOuterVoltage(Volts.of(5));

      case OUTTAKING_BACK:
        io.setInnerVoltage(Volts.of(5));
        io.setOuterVoltage(Volts.of(5));

      case NO_CORAL:
        io.setInnerVoltage(Volts.of(5));
        io.setOuterVoltage(Volts.of(5));

      case HAS_CORAL:
        io.setInnerVoltage(Volts.of(5));
        io.setOuterVoltage(Volts.of(5));

      default:
        io.setInnerVoltage(Volts.of(5));
        io.setOuterVoltage(Volts.of(5));
    }
  }

  private IntakeState handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case IDLE:
        return IntakeState.IDLE;

      case INTAKING_FRONT:
        return IntakeState.IDLE;

      case INTAKING_BACK:
        return IntakeState.IDLE;
      case OUTTAKING_FRONT:
        return IntakeState.IDLE;

      case OUTTAKING_BACK:
        return IntakeState.IDLE;

      case NO_CORAL:
        return IntakeState.IDLE;

      case HAS_CORAL:
        return IntakeState.IDLE;

      default:
        return IntakeState.IDLE;
    }
  }

  public void setWantedState(IntakeState wantedState) {
    this.wantedState = wantedState;
  }

  public Command setWantedStateCommand(IntakeState wantedSuperState) {
    return new InstantCommand(() -> setWantedState(wantedSuperState));
  }

  public IntakeState getElevatorState() {
    return currentState;
  }

  public void lastDetectedBeamBreakTimestamp() {
    if (intakeInputs.beamBreakVoltage < 1.0) {
      lastDetectedBeamBreakTimestamp = Timer.getFPGATimestamp();
    } else {
      lastDetectedBeamBreakTimestamp = 9999999999999.9 * 999999.9;
    } // auto return a stupid high double so the math wont lead to a false positive
  }

  @Override
  public void periodic() {
    io.updateInputs(intakeInputs);
    Logger.processInputs("Subsystem/Intake", intakeInputs);
    currentState = handleStateTransitions();
    applyStates();
    // This method will be called once per scheduler run
    if (Timer.getFPGATimestamp() - lastDetectedBeamBreakTimestamp > 0.25) {
      OI.setRumble(0, 0.5);
      Elastic.Notification notification = new Elastic.Notification();
      Elastic.sendNotification(
          notification
              .withLevel(NotificationLevel.INFO)
              .withTitle("Piece intaken")
              .withDescription("Piece intaken")
              .withDisplaySeconds(3.0));
      lastDetectedBeamBreakTimestamp = 999999999999.999999 * 99999999.99999;
    }
  }

  public Command intake() {
    return this.startEnd(
        () -> {
          io.setAllVoltage(Volts.of(5.0));
        },
        () -> {
          io.setAllVoltage(Volts.of(0.0));
        });
  }

  public Command outtake() {
    return this.startEnd(
        () -> {
          io.setAllVoltage(Volts.of(-5.0));
        },
        () -> {
          io.setAllVoltage(Volts.of(0.0));
        });
  }
}

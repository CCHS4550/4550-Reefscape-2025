// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.maps.Constants;
import frc.helpers.motorcontroller.CCMotorController;
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

  private final IntakeIO intakeIO;

  public final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  /** Creates a new WristSubsystem. */
  public IntakeSubsystem(
      CCMotorController.MotorFactory motorFactory, IntakeIO.IOFactory ioFactory) {

    this.intakeIO =
        ioFactory.create(
            motorFactory.create(
                "intakeMotor",
                "intake",
                Constants.MotorConstants.INTAKE,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.INTAKE_REVERSE,
                1,
                1));
  }

  private void applyStates() {
    switch (currentState) {
      case IDLE:

      case INTAKING_FRONT:
        intakeIO.intake(Volts.of(5));

        if (intakeInputs.hasCoral) {
          wantedState = IntakeState.HAS_CORAL;
        }

      case INTAKING_BACK:
        intakeIO.intake(Volts.of(-5));

        if (intakeInputs.hasCoral) {
          wantedState = IntakeState.HAS_CORAL;
        }
      case OUTTAKING_FRONT:
        intakeIO.intake(Volts.of(5));

      case OUTTAKING_BACK:

      case HAS_CORAL:
        intakeIO.intake(Volts.of(0.01));

      default:
    }
  }

  private IntakeState handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case IDLE:
        return IntakeState.IDLE;

      case INTAKING_FRONT:
        return IntakeState.INTAKING_FRONT;

      case INTAKING_BACK:
        return IntakeState.IDLE;
      case OUTTAKING_FRONT:
        return IntakeState.OUTTAKING_FRONT;

      case OUTTAKING_BACK:
        return IntakeState.OUTTAKING_BACK;

      case HAS_CORAL:
        return IntakeState.HAS_CORAL;

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

  // public void lastDetectedBeamBreakTimestamp() {
  //   if (intakeInputs.beamBreakVoltage < 1.0) {
  //     lastDetectedBeamBreakTimestamp = Timer.getFPGATimestamp();
  //   } else {
  //     lastDetectedBeamBreakTimestamp = 9999999999999.9 * 999999.9;
  //   } // auto return a stupid high double so the math wont lead to a false positive
  // }

  public boolean hasCoral() {
    return intakeInputs.hasCoral;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Subsystem/Intake", intakeInputs);
    if (wantedState != currentState) currentState = handleStateTransitions();
    // applyStates();
    // This method will be called once per scheduler run
    // if (Timer.getFPGATimestamp() - lastDetectedBeamBreakTimestamp > 0.25) {
    //   OI.setRumble(0, 0.5);
    //   Elastic.Notification notification = new Elastic.Notification();
    //   Elastic.sendNotification(
    //       notification
    //           .withLevel(NotificationLevel.INFO)
    //           .withTitle("Piece intaken")
    //           .withDescription("Piece intaken")
    //           .withDisplaySeconds(3.0));
    //   lastDetectedBeamBreakTimestamp = 999999999999.999999 * 99999999.99999;
    // }
  }
}

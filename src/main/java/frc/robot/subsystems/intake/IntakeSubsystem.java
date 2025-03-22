// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.util.maps.Constants;
import frc.util.motorcontroller.CCMotorController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public double lastDetectedBeamBreakTimestamp;

  private final IntakeIO intakeIO;

  public final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  private BooleanSupplier hasCoralSupplier = () -> intakeInputs.hasCoral;

  // private DoubleSupplier intakeSpeedModifier = () -> 1;

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

    processedCoralInput = false;
  }

  public Command intakeCoralStation() {
    return startEnd(() -> intakeIO.intake(Volts.of(12)), () -> intakeIO.intake(Volts.of(0)))
        .onlyWhile(hasCoralTrigger().negate());
  }

  public Command outtake() {
    return new FunctionalCommand(
        () -> {
          Timer timer = new Timer();
          timer.reset();
          timer.start();
          boolean outtakeReverse = Superstructure.shouldReverse();
          intakeIO.intake(Volts.of(outtakeReverse ? 12 : -12));
        },
        () -> {},
        (bool) -> {
          if (bool) intakeIO.intake(Volts.of(0));
        },
        () -> {
          return timer.hasElapsed(2);
        },
        this);
  }

  public Command intakeAuto() {
    return new FunctionalCommand(
        () -> intakeIO.intake(Volts.of(12)),
        () -> {},
        (bool) -> {
          intakeIO.intake(Volts.of(0));
        },
        () -> hasCoralSupplier.getAsBoolean(),
        this);
  }

  Timer endTimer = new Timer();
  boolean endStart = false;

  public Command outtakeAuto() {
    return new FunctionalCommand(
        () -> {
          boolean outtakeReverse = RobotState.getInstance().currentSuperState == SuperState.L4_BACK;
          intakeIO.intake(Volts.of(outtakeReverse ? -12 : 12));

          endStart = false;
          endTimer.reset();
          endTimer.stop();
        },
        () -> {
          if (!hasCoralSupplier.getAsBoolean() && !endStart) {
            endTimer.start();
            endStart = true;
          }
        },
        (bool) -> {
          intakeIO.intake(Volts.of(0));
        },
        () -> endTimer.hasElapsed(1),
        this);
  }

  public Command stop() {
    return runOnce(() -> intakeIO.intake(Volts.of(0)));
  }

  public boolean hasCoral() {
    return intakeInputs.hasCoral;
  }

  public Trigger hasCoralTrigger() {
    return new Trigger(() -> hasCoral());
  }

  Timer timer = new Timer();

  boolean processedCoralInput = false;

  public Trigger hasCoralDelayed(double delay) {
    if (hasCoral() && !processedCoralInput) {
      processedCoralInput = true;
      timer.reset();
      timer.start();
    } else if (!hasCoral() && processedCoralInput) {
      processedCoralInput = false;
      timer.stop();
    } else if (!hasCoral()) timer.stop();
    return new Trigger(() -> hasCoral() && timer.hasElapsed(delay));
  }

  // public void intakeNormal() {
  //   intakeSpeedModifier = () -> 1;
  // }

  // public void intakeSlow() {
  //   intakeSpeedModifier = () -> .5;
  // }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Subsystem/Intake", intakeInputs);
  }
}

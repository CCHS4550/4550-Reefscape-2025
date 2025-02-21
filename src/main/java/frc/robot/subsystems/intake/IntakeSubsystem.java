// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.maps.Constants;
import frc.helpers.motorcontroller.CCMotorController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public double lastDetectedBeamBreakTimestamp;

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

  public Command intakeCoralStation() {
    return startEnd(() -> intakeIO.intake(Volts.of(12)), () -> intakeIO.intake(Volts.of(0)));
  }

  public Command outtakeFront() {
    return runOnce(() -> intakeIO.intake(Volts.of(-12)));
  }

  public Command outtakeBack() {
    return runOnce(() -> intakeIO.intake(Volts.of(12)));
  }

  public boolean hasCoral() {
    return intakeInputs.hasCoral;
  }

  public BooleanSupplier hasCoralDelayed(double delay) {
    Timer timer = new Timer();
    if (intakeInputs.beamBroke) {
      timer.reset();
      timer.start();
    }
    return () -> timer.hasElapsed(delay);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Subsystem/Intake", intakeInputs);

    if (hasCoral()) intakeIO.intake(Volts.of(0));
  }
}

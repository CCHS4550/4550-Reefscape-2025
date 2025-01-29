// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCMotorReplay;
import frc.maps.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  /** Implementation of Singleton Pattern */

  private final ClimberIO climberIO;
  
  CCMotorController.MotorFactory motorFactory;
  ClimberIO.IOFactory ioFactory;

  public final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();


  /** Creates a new WristSubsystem. */
  public ClimberSubsystem(
      CCMotorController.MotorFactory motorFactory, ClimberIO.IOFactory ioFactory) {
    this.motorFactory = motorFactory;
    this.ioFactory = ioFactory;
    this.climberIO =
        ioFactory.create(
            motorFactory.create(
                "Climber Motor",
                "climb",
                Constants.MotorConstants.CLIMBER,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.CLIMBER_REVERSE,
                1,
                1));
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Subsystem/Climber", climberInputs);
    // This method will be called once per scheduler run
  }

  public Command climberUp() {
    return this.startEnd(
        () -> {
          climberIO.winchUp();
        },
        () -> {
          climberIO.setVoltage(Volts.of(0.0));
        });
  }

  public Command climberDown() {
    return this.startEnd(
        () -> {
          climberIO.winchDown();
        },
        () -> {
          climberIO.setVoltage(Volts.of(0.0));
        });
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class ElevatorSubsystem extends SubsystemBase {

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
              "elevatorMotor",
              "elevator",
              Constants.MotorConstants.ELEVATOR[0],
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.ELEVATOR_REVERSE,
              1,
              1));

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  public enum ElevatorPositions {
    // Placeholder Values
    STOW(Constants.ElevatorConstants.elevatorPositions[0]),
    L1(Constants.ElevatorConstants.elevatorPositions[1]),
    L2(Constants.ElevatorConstants.elevatorPositions[2]),
    L3(Constants.ElevatorConstants.elevatorPositions[3]),
    L4(Constants.ElevatorConstants.elevatorPositions[4]),
    A1(Constants.ElevatorConstants.elevatorPositions[5]),
    A2(Constants.ElevatorConstants.elevatorPositions[6]),
    CORAL_STATION(Constants.ElevatorConstants.elevatorPositions[7]),
    PROCESSOR(Constants.ElevatorConstants.elevatorPositions[8]);

    private final double heightMeters;

    ElevatorPositions(double heightMeters) {
      this.heightMeters = heightMeters;
    }

    public double getHeight() {
      return heightMeters;
    }
  }

  /** Implementation of Singleton Pattern */
  public static ElevatorSubsystem mInstance;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCSparkMax::new;
  private static ElevatorIO.IOFactory defaultIoFactory = ElevatorIOHardware::new;

  CCMotorController.MotorFactory motorFactory;
  ElevatorIO.IOFactory ioFactory;
  static ElevatorPositions currentPosition;

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
              1,
              1),
          motorFactory.create(
              "elevatorMotor2",
              "elevator2",
              Constants.MotorConstants.ELEVATOR[1],
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.ELEVATOR_REVERSE[1],
              1,
              1));

  // @Override
  public void changeElevatorPosition(ElevatorPositions desiredPosition) {
    currentPosition = desiredPosition;
  }

  public static double heightToRotations(double height) {
    return height * 0.2982; // random value
  }

  public static double rotationsToHeight(double rotations) {
    return rotations * 0.8263;
  }

  public static ElevatorPositions getElevatorPosition() {
    return currentPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

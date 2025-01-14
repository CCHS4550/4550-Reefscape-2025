// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
import frc.robot.subsystems.Wrist.WristIOInputsAutoLogged;

public class WristSubsystem extends SubsystemBase {

  public enum WristPositions {
    // Placeholder Values
    L1_FRONT(30),
    L2L3_FRONT(45),
    L4_BACK(110),
    CORAL_STATION_FRONT(15),
    CORAL_STATION_BACK(120);

    private final double angleDegrees;

    WristPositions(double angleDegrees) {
      this.angleDegrees = angleDegrees;
    }
  }

  /** Implementation of Singleton Pattern */
  public static WristSubsystem mInstance;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCSparkMax::new;
  private static WristIO.IOFactory defaultIoFactory = WristIOHardware::new;

  CCMotorController.MotorFactory motorFactory;
  WristIO.IOFactory ioFactory;

  WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

  /** Singleton Practice */
  public static WristSubsystem getInstance(
      CCMotorController.MotorFactory motorFactory, WristIO.IOFactory ioFactory) {
    if (mInstance == null) {
      mInstance = new WristSubsystem(motorFactory, ioFactory);
    }
    return mInstance;
  }

  /** Singleton Practice */
  public static WristSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new WristSubsystem(defaultMotorFactory, defaultIoFactory);
    }
    return mInstance;
  }

  /** Creates a new WristSubsystem. */
  private WristSubsystem(CCMotorController.MotorFactory motorFactory, WristIO.IOFactory ioFactory) {
    this.motorFactory = motorFactory;
    this.ioFactory = ioFactory;
  }

  private final WristIO io =
      ioFactory.create(
          motorFactory.create(
              "wristMotor",
              "wrist",
              Constants.MotorConstants.WRIST,
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.WRIST_REVERSE,
              1,
              1));

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

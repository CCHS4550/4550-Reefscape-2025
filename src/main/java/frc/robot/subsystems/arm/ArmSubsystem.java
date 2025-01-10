// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkMax;



public class ArmSubsystem extends SubsystemBase {

  /** Implementation of Singleton Pattern */
  public static ArmSubsystem mInstance;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCSparkMax::new;
  private static ArmIO.IOFactory defaultIoFactory = ArmIOHardware::new;

  CCMotorController.MotorFactory motorFactory;
  ArmIO.IOFactory ioFactory;

  public static ArmSubsystem getInstance(CCMotorController.MotorFactory motorFactory, ArmIO.IOFactory ioFactory) {
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


  public enum ArmPositions {
    // Placeholder Values
    L1_FRONT(30),
    L2L3_FRONT(45),
    L4_BACK(110),
    CORAL_STATION_FRONT(15),
    CORAL_STATION_BACK(120);

    private final double angleDegrees;

    ArmPositions(double angleDegrees) {
      this.angleDegrees = angleDegrees;
    }

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

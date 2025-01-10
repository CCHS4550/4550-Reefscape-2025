// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkMax;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOHardware;
import frc.robot.subsystems.wrist.WristSubsystem;


public class IntakeSubsystem extends SubsystemBase {

  /** Implementation of Singleton Pattern */
  public static IntakeSubsystem mInstance;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCSparkMax::new;
  private static IntakeIO.IOFactory defaultIoFactory = IntakeIOHardware::new;

  CCMotorController.MotorFactory motorFactory;
  IntakeIO.IOFactory ioFactory;

  public static IntakeSubsystem getInstance(CCMotorController.MotorFactory motorFactory, IntakeIO.IOFactory ioFactory) {
    if (mInstance == null) {
      mInstance = new IntakeSubsystem(motorFactory, ioFactory);
    }
    return mInstance;
  }

  public static IntakeSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new IntakeSubsystem(defaultMotorFactory, defaultIoFactory);
    }
    return mInstance;
  }

  /** Creates a new WristSubsystem. */
  private IntakeSubsystem(CCMotorController.MotorFactory motorFactory, IntakeIO.IOFactory ioFactory) {
      this.motorFactory = motorFactory;
      this.ioFactory = ioFactory; 

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

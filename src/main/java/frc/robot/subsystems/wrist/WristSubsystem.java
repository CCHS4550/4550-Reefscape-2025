// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkMax;
import frc.robot.subsystems.wrist.WristIO;

public class WristSubsystem extends SubsystemBase {
  
  /** Implementation of Singleton Pattern */
  public static WristSubsystem mInstance;


  private static CCMotorController.MotorFactory defaultMotorFactory = CCSparkMax::new;
  private static WristIO.IOFactory defaultIoFactory = WristIOHardware::new;

  CCMotorController.MotorFactory motorFactory;
  WristIO.IOFactory ioFactory;

  public static WristSubsystem getInstance(CCMotorController.MotorFactory motorFactory, WristIO.IOFactory ioFactory) {
    if (mInstance == null) {
      mInstance = new WristSubsystem(motorFactory, ioFactory);
    }
    return mInstance;
  }

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

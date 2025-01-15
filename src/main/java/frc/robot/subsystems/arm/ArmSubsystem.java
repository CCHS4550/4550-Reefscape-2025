// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class ArmSubsystem extends SubsystemBase {

  public enum ArmState {
    // Placeholder Values
    DEFAULT_WITHINFRAME(Units.degreesToRadians(0)),
    L1_FRONT(Units.degreesToRadians(30)),
    L2L3_FRONT(Units.degreesToRadians(45)),
    L4_BACK(Units.degreesToRadians(110)),
    CORAL_STATION_FRONT(Units.degreesToRadians(15)),
    CORAL_STATION_BACK(Units.degreesToRadians(120));

    public final double angleDegrees;

    ArmState(double angleDegrees) {
      this.angleDegrees = angleDegrees;
    }

    public double getAngle() {
      return angleDegrees;
    }
  }

  public ArmState previousState = ArmState.DEFAULT_WITHINFRAME;
  public ArmState cuurrentState = ArmState.DEFAULT_WITHINFRAME;
  public ArmState wantedState = ArmState.DEFAULT_WITHINFRAME;


  /** Implementation of Singleton Pattern */
  public static ArmSubsystem mInstance;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCSparkMax::new;
  private static ArmIO.IOFactory defaultIoFactory = ArmIOHardware::new;

  CCMotorController.MotorFactory motorFactory;
  ArmIO.IOFactory ioFactory;

  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  public static ArmSubsystem getInstance(
      CCMotorController.MotorFactory motorFactory, ArmIO.IOFactory ioFactory) {
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

  private final ArmIO io =
      ioFactory.create(
          motorFactory.create(
              "armMotor",
              "arm",
              Constants.MotorConstants.ARM,
              MotorType.kBrushless,
              IdleMode.kBrake,
              Constants.MotorConstants.ARM_REVERSE,
              1,
              1));

  
    private void applyStates() {
    }
    private void handleStateTransitions() {
      switch(wantedState) {
        
      }
    }
    

  @Override
  public void periodic() {
    io.updateInputs(armInputs);
    handleStateTransitions();

    // This method will be called once per scheduler run
  }
}

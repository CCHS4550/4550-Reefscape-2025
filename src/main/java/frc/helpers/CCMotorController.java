// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.helpers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface CCMotorController {

  default void reset() {}

  default void set(double speed) {}

  default void set(boolean stop, double speed) {}

  default void setVoltage(double volts) {}

  default void setVoltage(double volts, double currentlimit) {}

  default double getVoltage() {
    return 0;
  }

  default void setVoltageFromSpeed(double speed) {}

  /* The actual speed in whatever units. */
  default double getVelocity() {
    return 0;
  }

  default double getRawVelocity() {
    return 0;
  }

  /* What you set it to. */
  default double getSpeed() {
    return 0;
  }

  default void setPositionConversionFactor(double factor) {}

  default void setVelocityConversionFactor(double factor) {}

  default void setPosition(double pos) {}

  default double getPosition() {
    return 0;
  }

  default double getRawPosition() {
    return 0;
  }

  default Object getEncoder() {
    return null;
  }
  default Object getAlternateEncoder() {
    return null;
  }

  default String getName() {
    return "No Given Name";
  }

  default String getShortName() {
    return "No Given ShortName";
  }

  @AutoLog
  class MotorData {
    public double position = 0;
    public double setOutput = 0;
    public double velocity = 0;

    public double voltage = 0;

    public double speed = 0;
  }

  // public static <T> void createObject(ObjectFactory<T> factory) {
  //   T obj = factory.create(); // Use the factory to create an object
  //   System.out.println("Object created: " + obj);
  // }

  // @FunctionalInterface
  // interface ObjectFactory<T> {
  //   T create(); // Abstract method to match no-arg constructor
  // }

  @FunctionalInterface
  public interface MotorFactory {
    CCMotorController create(
        String name,
        String shortName,
        int deviceID,
        MotorType motorType,
        IdleMode idleMode,
        boolean reverse,
        double positionConversionFactor,
        double velocityConversionFactor);
  }

  // @FunctionalInterface
  // interface MotorFactory {
  // CCMotorController create(Object... args);
  // }

}

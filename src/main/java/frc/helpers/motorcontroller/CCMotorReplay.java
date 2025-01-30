// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.helpers.motorcontroller;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class CCMotorReplay implements CCMotorController {

  // Don't mess with this file

  public CCMotorReplay(
      String name,
      String shortName,
      int deviceID,
      MotorType motorType,
      IdleMode idleMode,
      boolean reverse,
      double positionConversionFactor,
      double velocityConversionFactor) {}

  @Override
  public void reset() {}

  @Override
  public void set(double speed) {}

  @Override
  public void set(boolean stop, double speed) {}

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void setVoltage(double volts, double currentlimit) {}

  @Override
  public double getVoltage() {
    return 0;
  }

  @Override
  public void setVoltageFromSpeed(double speed) {}

  /* The actual speed in whatever units. */
  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public double getRawVelocity() {
    return 0;
  }

  /* What you set it to. */
  @Override
  public double getSpeed() {
    return 0;
  }

  @Override
  public void setPositionConversionFactor(double factor) {}

  @Override
  public void setVelocityConversionFactor(double factor) {}

  @Override
  public void setPosition(double pos) {}

  @Override
  public double getPosition() {
    return 0;
  }

  @Override
  public double getRawPosition() {
    return 0;
  }

  @Override
  public Object getEncoder() {
    return null;
  }

  @Override
  public String getName() {
    return "No Given Name";
  }

  @Override
  public String getShortName() {
    return "No Given ShortName";
  }
}

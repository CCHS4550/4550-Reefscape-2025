// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class CCSparkSim extends SparkMaxSim implements CCMotorController {

  // Constants for a NEO motor.
  private static final double nominalVoltageVolts = 12;
  private static final double stallTorqueNewtonMeters = 2.6;
  private static final double stallCurrentAmps = 105;
  private static final double freeCurrentAmps = 1.8;
  private static final double freeSpeedRadPerSec = 594.39;
  private static final int numMotors = 1;

  private String name;
  private String shortName;
  private SparkMaxAlternateEncoderSim encoder;
  private double voltageConversionFactor;
  private double velocityConversionFactorOne = 1.0;
  private double positionConversionFactorOne = 1.0;

  // This will error but it should work nonetheless.
  MotorDataAutoLogged simMotor = new MotorDataAutoLogged();

  public CCSparkSim(
      String name,
      String shortName,
      int deviceID,
      MotorType motorType,
      IdleMode idleMode,
      boolean reverse,
      double positionConversionFactor,
      double velocityConversionFactor) {
    super(
        createConfiguredSparkMax(deviceID, motorType, reverse, idleMode),
        new DCMotor(
            nominalVoltageVolts,
            stallTorqueNewtonMeters,
            stallCurrentAmps,
            freeCurrentAmps,
            freeSpeedRadPerSec,
            numMotors));

    this.name = name;
    this.shortName = shortName;

    this.encoder = super.getAlternateEncoderSim();
    this.setPositionConversionFactor(positionConversionFactor);
    this.setVelocityConversionFactor(velocityConversionFactor);
    voltageConversionFactor = 12;
  }

  /**
   * Helper class intended to create a SparkMax and configure it simultaneously.
   *
   * @param deviceID CAN ID
   * @param motorType Brushed or Brushless
   * @param reverse true or false
   * @param idleMode Brake or Coast
   * @return
   */
  private static SparkMax createConfiguredSparkMax(
      int deviceID, MotorType motorType, boolean reverse, IdleMode idleMode) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(reverse).idleMode(idleMode);

    SparkMax sparkMax = new SparkMax(deviceID, motorType);
    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return sparkMax;
  }

  // public CCSparkSim(
  //     String name,
  //     String shortName,
  //     int deviceID,
  //     MotorType motorType,
  //     IdleMode idleMode,
  //     boolean reverse) {
  //         super(
  //             createConfiguredSparkMax(deviceID, motorType, reverse, idleMode),
  //             new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters, stallCurrentAmps,
  // freeCurrentAmps, freeSpeedRadPerSec, numMotors));

  //         this.name = name;
  //         this.shortName = shortName;

  //         this.encoder = super.getAlternateEncoderSim();

  //         voltageConversionFactor = 12;
  // }

  @Override
  public void reset() {
    encoder.setPosition(0);
    simMotor.position = 0;
  }

  /**
   * Sets the speed of the motor controller
   *
   * @param speed The speed that will be set (-1.0 to 1.0)
   */
  @Override
  public void set(double speed) {
    super.setAppliedOutput(speed);
    simMotor.speed = speed;
  }

  @Override
  public void setVoltage(double volts) {
    super.setBusVoltage(volts);
  }

  @Override
  public void setVoltage(double volts, double currentlimit) {
    if (super.getMotorCurrent() <= currentlimit) {
      setVoltage(volts);
    } else {
      setVoltage(0);
    }
  }

  @Override
  public void setVoltageFromSpeed(double speed) {
    setVoltage(speed * voltageConversionFactor);
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity() * velocityConversionFactorOne;
  }

  @Override
  public void disable() {
    super.disable();
  }

  @Override
  public double getSpeed() {
    return super.getAppliedOutput();
  }

  /**
   * Sets the Position Conversion Factor for the encoder
   *
   * @param factor The ratio of encoder units to desired units (ie. units -> in)
   */
  @Override
  public void setPositionConversionFactor(double factor) {
    // encoder.setPositionConversionFactor(factor);
    positionConversionFactorOne = factor;
  }

  /**
   * Sets the Velocity Conversion Factor for the encoder
   *
   * @param factor The ratio of encoder units to desired units (ie. units/min-> rad/sec)
   */
  @Override
  public void setVelocityConversionFactor(double factor) {
    // encoder.setVelocityConversionFactor(factor);
    velocityConversionFactorOne = factor;
  }

  /**
   * Sets the encoder position
   *
   * @param pos The new encoder position
   */
  @Override
  public void setPosition(double pos) {
    encoder.setPosition(pos);
  }

  /**
   * Returns the position of the encoder. By default the position is in encoder units, but will
   * return a distance if the Position Conversion Factor has been set.
   */
  @Override
  public double getPosition() {
    return encoder.getPosition() * positionConversionFactorOne;
  }

  @Override 
  public SparkMaxAlternateEncoderSim getEncoder() {
    return encoder;
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public String getShortName() {
    return shortName;
  }




  @Override
  public void set(boolean stop, double speed) {
    if (!stop) super.setAppliedOutput(speed);
  }
}

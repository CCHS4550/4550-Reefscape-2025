package frc.helpers.motorcontroller;

import com.revrobotics.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// Documention: https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html

public class CCSparkMax extends SparkMax implements CCMotorController {

  SparkMaxConfig config;
  private String name;
  private String shortName;
  private RelativeEncoder encoder;
  private RelativeEncoder alternateEncoder = null;
  private AlternateEncoderConfig encoderConfig;
  private AbsoluteEncoder absoluteEncoder = null;
  private AbsoluteEncoderConfig absoluteEncoderConfig;

  private double voltageConversionFactor;
  private double velocityConversionFactor = 1.0;
  private double positionConversionFactor = 1.0;

  ResetMode resetMode = ResetMode.kResetSafeParameters;
  PersistMode persistMode = PersistMode.kPersistParameters;

  // This will error but it should work nonetheless.
  MotorDataAutoLogged realMotor = new MotorDataAutoLogged();

  /**
   * CCSparkMax allows us to easily control Spark Max motor controllers Information on modes can be
   * found in the Spark Max documentation
   *
   * @param deviceID The CAN channel of the motor controller
   * @param controlMode Specify whether the motor controller is operating in Brushed or Brushless
   *     mode
   * @param idleMode Specify whether the motor controller is set to Coast or Brake mode
   * @param reverse Reverses the direction of the motor controller
   * @param encoder If the motor has an encoder or not
   * @param positionConversionFactor Conversion rate for position from rotations to desired unit
   * @param velocityConversionFactor Conversion rate for velocity from rotations per minute to
   *     desired unit
   */
  public CCSparkMax(
      String name,
      String shortName,
      int deviceID,
      MotorType motorType,
      IdleMode idleMode,
      boolean reverse,
      double positionConversionFactor,
      double velocityConversionFactor) {
    super(deviceID, motorType);
    this.name = name;
    this.shortName = shortName;

    config = new SparkMaxConfig();

    config.inverted(reverse).idleMode(idleMode);

    this.encoder = super.getEncoder();

    this.setPositionConversionFactor(positionConversionFactor);
    this.setVelocityConversionFactor(velocityConversionFactor);
    voltageConversionFactor = 12;

    super.configure(config, resetMode, persistMode);
  }

  //   public CCSparkMax(
  //       String name,
  //       String shortName,
  //       int deviceID,
  //       MotorType motorType,
  //       IdleMode idleMode,
  //       boolean reverse) {
  //     // super(deviceID, motorType);
  //     super(deviceID, motorType);
  //     this.name = name;
  //     this.shortName = shortName;

  //     super.setInverted(reverse);
  //     super.setIdleMode(idleMode);

  //     this.encoder = super.getEncoder();
  //     // Will return a value in Radians
  //     // this.setPositionConversionFactor(2 * Math.PI);
  //     // // Will return a value in Radians per Second
  //     // this.setVelocityConversionFactor(2 * Math.PI / 60);
  //     voltageConversionFactor = 12;
  //     super.burnFlash();
  // }

  // public CCSparkMax(
  //     String name,
  //     String shortName,
  //     int deviceID,
  //     MotorType motorType,
  //     IdleMode idleMode,
  //     boolean reverse) {
  //   super(deviceID, motorType);
  //   this.name = name;
  //   this.shortName = shortName;
  //   SparkMaxConfig config = new SparkMaxConfig();
  //   config.inverted(reverse).idleMode(idleMode);

  //   this.encoder = super.getEncoder();
  //   voltageConversionFactor = 12;

  //   super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  // }

  @Override
  public void reset() {
    encoder.setPosition(0);
    realMotor.position = 0;
  }

  /**
   * Sets the speed of the motor controller
   *
   * @param speed The speed that will be set (-1.0 to 1.0)
   */
  @Override
  public void set(double speed) {
    super.set(speed);
    realMotor.speed = speed;
  }

  @Override
  public void setVoltage(double volts) {
    super.setVoltage(volts);
    realMotor.voltage = volts;
  }

  @Override
  public void setVoltage(double volts, double currentlimit) {
    if (super.getOutputCurrent() <= currentlimit) {
      super.setVoltage(volts);
      realMotor.voltage = volts;
    } else {
      super.setVoltage(0);
      realMotor.voltage = 0;
    }
  }

  @Override
  public REVLibError getLastError() {
    // TODO Auto-generated method stub
    return super.getLastError();
  }

  @Override
  public double getVoltage() {
    return realMotor.voltage;
  }

  @Override
  public double getCurrent() {
    return super.getOutputCurrent();
  }

  @Override
  public void setVoltageFromSpeed(double speed) {
    super.setVoltage(speed * voltageConversionFactor);
    realMotor.voltage = speed * voltageConversionFactor;
  }

  @Override
  public double getVelocity() {
    if (alternateEncoder != null) alternateEncoder.getVelocity();
    if (absoluteEncoder != null) return absoluteEncoder.getVelocity();
    return encoder.getVelocity() * velocityConversionFactor;
  }

  @Override
  public void disable() {
    super.disable();
  }

  @Override
  public double getSpeed() {
    return super.get();
  }

  /**
   * Sets the Position Conversion Factor for the encoder
   *
   * @param factor The ratio of encoder units to desired units (ie. units -> in)
   */
  @Override
  public void setPositionConversionFactor(double factor) {
    if (alternateEncoder != null) {
      encoderConfig.positionConversionFactor(factor);
      config.apply(encoderConfig);
      configure(config, resetMode, persistMode);
    }
    if (absoluteEncoder != null) {
      absoluteEncoderConfig.positionConversionFactor(factor);
      config.apply(absoluteEncoderConfig);
      configure(config, resetMode, persistMode);
    }
    this.positionConversionFactor = factor;
  }

  /**
   * Sets the Velocity Conversion Factor for the encoder
   *
   * @param factor The ratio of encoder units to desired units (ie. units/min-> rad/sec)
   */
  @Override
  public void setVelocityConversionFactor(double factor) {
    if (alternateEncoder != null) {
      encoderConfig.velocityConversionFactor(factor);
      config.apply(encoderConfig);
      configure(config, resetMode, persistMode);
    }
    if (absoluteEncoder != null) {
      absoluteEncoderConfig.velocityConversionFactor(factor);
      config.apply(absoluteEncoderConfig);
      configure(config, resetMode, persistMode);
    }
    this.velocityConversionFactor = factor;
  }

  /**
   * Sets the encoder position
   *
   * @param pos The new encoder position
   */
  @Override
  public void setPosition(double pos) {
    encoder.setPosition(pos);
    realMotor.position = pos;
  }

  /**
   * Returns the position of the encoder. By default the position is in encoder units, but will
   * return a distance if the Position Conversion Factor has been set.
   */
  @Override
  public double getPosition() {
    if (alternateEncoder != null) return alternateEncoder.getPosition();

    if (absoluteEncoder != null) return absoluteEncoder.getPosition();

    return encoder.getPosition() * positionConversionFactor;
  }

  @Override
  public RelativeEncoder getEncoder() {
    return encoder;
  }

  @Override
  public RelativeEncoder getAlternateEncoder() {

    encoderConfig = new AlternateEncoderConfig();
    // encoderConfig.positionConversionFactor(positionConversionFactor);
    // encoderConfig.velocityConversionFactor(velocityConversionFactor);
    encoderConfig.setSparkMaxDataPortConfig();
    config.apply(encoderConfig);
    configure(config, resetMode, persistMode);

    this.alternateEncoder = super.getAlternateEncoder();
    return alternateEncoder;
  }

  @Override
  public AbsoluteEncoder getDataportAbsoluteEncoder() {
    absoluteEncoderConfig = new AbsoluteEncoderConfig();
    absoluteEncoderConfig.setSparkMaxDataPortConfig();
    config.apply(absoluteEncoderConfig);
    this.absoluteEncoder = super.getAbsoluteEncoder();
    return absoluteEncoder;
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
    if (!stop) super.set(speed);
  }
}

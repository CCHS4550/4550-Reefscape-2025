package frc.robot.helpers;


import com.revrobotics.*;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// Documention: https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html

public class CCSparkMax extends SparkMax {

  private String name;
  private String shortName;
  private RelativeEncoder encoder;
  private double voltageConversionFactor;
  private double velocityConversionFactorOne = 1.0;
  private double positionConversionFactorOne = 1.0;

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
    SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(reverse)
            .idleMode(idleMode);


    this.encoder = super.getEncoder();
    this.setPositionConversionFactor(positionConversionFactor);
    this.setVelocityConversionFactor(velocityConversionFactor);
    voltageConversionFactor = 12;

    super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
//}
    public CCSparkMax(
      String name,
      String shortName,
      int deviceID,
      MotorType motorType,
      IdleMode idleMode,
      boolean reverse) {
    super(deviceID, motorType);
    this.name = name;
    this.shortName = shortName;
    SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(reverse)
            .idleMode(idleMode);


    this.encoder = super.getEncoder();
    voltageConversionFactor = 12;

    super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

//   public CCSparkMax(
//       String name   ,
//       String shortName,
//       int deviceID,
//       MotorType motorType,
//       IdleMode idleMode,
//       boolean reverse,
//       double encoder) {
//     super(deviceID, motorType);
//     this.name = name;
//     this.shortName = shortName;

//     super.setInverted(reverse);
//     super.setIdleMode(idleMode);

//     if (encoder < 0) return;
//     this.encoder = super.getEncoder();

//     // Will return a value in Radians
//     // this.setPositionConversionFactor(2 * Math.PI);
//     // // Will return a value in Radians per Second
//     // this.setVelocityConversionFactor(2 * Math.PI / 60);
//     voltageConversionFactor = 12;
//     super.burnFlash();
//   }
     public CCSparkMax(
      String name,
      String shortName,
      int deviceID,
      MotorType motorType,
      IdleMode idleMode,
      boolean reverse, 
      double encoder) {
    super(deviceID, motorType);
    this.name = name;
    this.shortName = shortName;
    SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(reverse)
            .idleMode(idleMode);


     if (encoder < 0) return;
     this.encoder = super.getEncoder();

//     // Will return a value in Radians
//     // this.setPositionConversionFactor(2 * Math.PI);
//     // // Will return a value in Radians per Second
//     // this.setVelocityConversionFactor(2 * Math.PI / 60);
//     voltageConversionFactor = 12;
    super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

//   public CCSparkMax(
//       String name,
//       String shortName,
//       int deviceID,
//       MotorType motorType,
//       IdleMode idleMode,
//       boolean reverse,
//       boolean encoder) {
//     super(deviceID, motorType);
//     this.name = name;
//     this.shortName = shortName;

//     super.setInverted(reverse);
//     super.setIdleMode(idleMode);

//     voltageConversionFactor = 12;
//     super.burnFlash();
//   }
//     public CCSparkMax(
//       String name,
//       String shortName,
//       int deviceID,
//       MotorType motorType,
//       IdleMode idleMode,  
//       boolean reverse,
//       double encoder) {
//     super(deviceID, motorType);
//     this.name = name;
//     this.shortName = shortName;
//     SparkMaxConfig config = new SparkMaxConfig();
//         config
//             .inverted(reverse)
//             .idleMode(idleMode);
//     voltageConversionFactor = 12;

//     super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//   }
  public void reset() {
    encoder.setPosition(0);
  }

  /**
   * Sets the speed of the motor controller
   *
   * @param speed The speed that will be set (-1.0 to 1.0)
   */
  public void set(double speed) {
    super.set(speed);
  }

  public void setVoltage(double volts) {
    super.setVoltage(volts);
  }

  public void setVoltage(double volts, double currentlimit) {
    if (super.getOutputCurrent() <= currentlimit) {
      super.setVoltage(volts);
    } else {
      super.setVoltage(0);
    }
  }

  public void setVoltageFromSpeed(double speed) {
    super.setVoltage(speed * voltageConversionFactor);
  }

  public double getVelocity() {
    return encoder.getVelocity()*velocityConversionFactorOne;
  }

  public void disable() {
    super.disable();
  }

  public double getSpeed() {
    return super.get();
  }

  /**
   * Sets the Position Conversion Factor for the encoder
   *
   * @param factor The ratio of encoder units to desired units (ie. units -> in)
   */
  public void setPositionConversionFactor(double factor) {
    // encoder.setPositionConversionFactor(factor);
    positionConversionFactorOne = factor;
  }

  /**
   * Sets the Velocity Conversion Factor for the encoder
   *
   * @param factor The ratio of encoder units to desired units (ie. units/min-> rad/sec)
   */
  public void setVelocityConversionFactor(double factor) {
    // encoder.setVelocityConversionFactor(factor);
    velocityConversionFactorOne = factor;
  }

  /**
   * Sets the encoder position
   *
   * @param pos The new encoder position
   */
  public void setPosition(double pos) {
    encoder.setPosition(pos);
  }

  /**
   * Returns the position of the encoder. By default the position is in encoder units, but will
   * return a distance if the Position Conversion Factor has been set.
   */   
  public double getPosition() {
    return encoder.getPosition()*positionConversionFactorOne;
  }

  /**
   * Sets the PID values, must be positive
   *
   * @param Kp The proportional gain value
   * @param Ki The integral gain value
   * @param Kd The derivative gain value
   */
  public String getName() {
    return name;
  }

  public String getShortName() {
    return shortName;
  }

  public void set(boolean stop, double speed) {
    if (!stop) super.set(speed);
  }
}
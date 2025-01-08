package frc.robot.subsystems.DriveTrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.AnalogEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.CCSparkMax;
import frc.robot.helpers.SparkMaxBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

// import org.littletonrobotics.junction.Logger;

/**
 * Class for controlling a swerve module. Each module has 2 motors, one for driving and one for
 * turning, as well as an absolute encoder.
 *
 * <p>Swerve modules are set to different positions and vectors in the SwerveModuleState format.
 * SwerveModuleState takes in a drive speed in meters per second and an angle in radians in the
 * format of Rotation2d.
 */
public class SwerveModule extends SubsystemBase {
  private CCSparkMax driveMotor;
  private CCSparkMax turnMotor;

  private PIDController turningPIDController, drivingPidController;
  private SimpleMotorFeedforward driveFeedforward /* , turnFeedforward */;

  // private SparkPIDController turningPIDController;

  private AnalogEncoder absoluteEncoder;
  private double absoluteEncoderOffset;
  private String name;

  // adjust absoluteEncoderChannel to possibly be absoluteEncoderAnalogInput
  /**
   * Creates a SwerveModule object with a defined drive motor, turn motor, and absolute encoder.
   *
   * @param driveMotor The drive motor in CCSparkMax format.
   * @param turnMotor The turn motor in CCSparkMax format.
   * @param absoluteEncoderChannel The port of the absolute encoder.
   * @param absoluteEncoderOffset The offset of the absolute encoder in radians.
   */
  public SwerveModule(
      SparkMaxBase driveMotor,
      SparkMaxBase turnMotor,
      int absoluteEncoderChannel,
      double absoluteEncoderOffset,
      String name) {
    this.driveMotor = driveMotor;
    this.turnMotor = turnMotor;

    this.absoluteEncoder = new AnalogEncoder(absoluteEncoderChannel);

    this.absoluteEncoder.setDistancePerRotation(2 * Math.PI);

    // this.absoluteEncoder.setPositionOffset(absoluteEncoderOffset);
    this.absoluteEncoderOffset = absoluteEncoderOffset;
    // turningPIDController = new SparkPIDController(.5, 0, 0);
    // turningPIDController = new SparkPIDController();

    turningPIDController = new PIDController(0.4, 0, 0);
    turningPIDController.enableContinuousInput(0, 2 * Math.PI);
    drivingPidController = new PIDController(1, 0, 0);

    // possibly kA values too, if sysid provides those
    driveFeedforward =
        new SimpleMotorFeedforward(
            Constants.FeedForwardConstants.DRIVE_KS,
            Constants.FeedForwardConstants.DRIVE_KV,
            Constants.FeedForwardConstants.DRIVE_KA);

    this.name = name;
    resetEncoders();
  }

  /**
   * Gets the encoder value of the drive motor in meters.
   *
   * @return The encoder value of the drive motor.
   */
  public double getDrivePosition() {
    return driveMotor.getPosition(); // should be in meters?
  }

  public SparkAnalogSensor getDriveAnalog() {
    return driveMotor.getAnalog();
  }

  /**
   * Gets the encoder value of the turn motor.
   *
   * @return The encoder value of the turn motor.
   */
  public double getTurnPosition() {
    // return turnMotor.getPosition() % (2 * Math.PI); // should be in radians?
    return getAbsoluteEncoderRadiansOffset();
  }

  public double getDriveVoltagee() {
    return driveMotor.getAppliedOutput();
  }

  /**
   * Gets the speed of the turn motor.
   *
   * @return The speed of the turn motor between -1 and 1.
   */
  public double getTurnVelocity() {
    return turnMotor.getSpeed();
  }

  /**
   * Gets the voltage being supplied to the drive motor.
   *
   * @return The voltage being supplied to the drive motor.
   */
  public double getDriveVoltage() {
    return driveMotor.get() * driveMotor.getBusVoltage();
  }

  /**
   * Gets the voltage being supplied to the turn motor.
   *
   * @return The voltage being supplied to the turn motor.
   */
  public double getTurnVoltage() {
    return turnMotor.get() * turnMotor.getBusVoltage();
  }

  /**
   * Gets the reading of the absolute encoder with offset.
   *
   * @return The value of the absolute encoder in radians with the offset applied.
   */
  public double getAbsoluteEncoderRadiansOffset() {
    return Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition())
        - absoluteEncoderOffset
        + Math.PI;
  }

  /**
   * Gets the reading of the absolute encoder with offset. Used for getting the offset.
   *
   * @return The value of the absolute encoder in radians without the offset applied.
   */
  public double getAbsoluteEncoderRadiansNoOffset() {
    return Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition());
  }

  /**
   * Resets the drive and turn motor encoders. The drive motor is set to 0 while the turn motor is
   * set to the value of the absolute encoder.
   */
  public void resetEncoders() {
    driveMotor.reset();
    turnMotor.setPosition(getAbsoluteEncoderRadiansOffset());
  }

  public void resetTurnEncoder() {
    turnMotor.setPosition(getAbsoluteEncoderRadiansOffset());
  }

  /**
   * Gets the state of the module.
   *
   * @return The state of the swerve module in SwerveModuleState format.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveEncoderVelocity(), new Rotation2d(getTurnPosition()));
  }

  /**
   * Sets the state of the module.
   *
   * @param state The state to set the swerve module to in SwerveModuleState format.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (Math.abs(desiredState.speedMetersPerSecond) <= .005) {
      stop();
      return;
    }

    Rotation2d encoderRotation = new Rotation2d(getState().angle.getRadians());

    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
    // Minimizes side drift when driving
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    setDriveVelocity(state.speedMetersPerSecond);
    Logger.recordOutput("desiredState - Meters per Second", state.speedMetersPerSecond);
    setTurnPosition(() -> state.angle.getRadians());
    // setTurnPosition();

  }

  public void setDriveVelocity(double velocity) {
    // These are both in m/s
    double driveOutput =
        drivingPidController.calculate(driveMotor.getEncoder().getVelocity(), velocity);
    Logger.recordOutput("desired drivePID Output", driveOutput);
    // Feed forward
    double driveFF = driveFeedforward.calculate(velocity);
    Logger.recordOutput("desired driveFF Output", driveFF);

    driveMotor.setVoltage(driveOutput + driveFF);
    Logger.recordOutput("desired drivePID + driveFF Output", driveOutput + driveFF);
  }

  public void setTurnPosition(DoubleSupplier angle) {
    double turnOutput =
        turningPIDController.calculate(getAbsoluteEncoderRadiansOffset(), angle.getAsDouble());
    // turnMotor.setVoltage(turnOutput);
    turnMotor.set(turnOutput);
    /* TODO Change back */
  }

  /** Sets the speed of the drive and turn motors to 0. */
  public void stop() {
    driveMotor.setVoltageFromSpeed(0);
    turnMotor.setVoltageFromSpeed(0);
  }

  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
  }

  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
  }

  /**
   * Method for testing purposes
   *
   * @param driveSpeed Speed of the drive motor.
   * @param turnSpeed Speed of the turn motor.
   */
  public void driveAndTurn(double driveSpeed, double turnSpeed) {
    driveMotor.setVoltageFromSpeed(driveSpeed);
    turnMotor.setVoltageFromSpeed(turnSpeed);
  }

  public void printEncoders() {
    System.out.println(
        name
            + "\nDrive Encoder: "
            + driveMotor.getPosition()
            + "\nTurn Encoder: "
            + turnMotor.getPosition()
            + "\n");
  }

  public void resetAbsoluteEncoder() {
    absoluteEncoder.reset();
  }

  public void printAbsoluteEncoder() {
    System.out.println(name + ": " + absoluteEncoder.getDistance());
  }

  public String getName() {
    return name;
  }

  public void setName(String name) {
    this.name = name;
  }

  public double getTurnEncoderDistance() {
    return turnMotor.getPosition();
  }

  public double getTurnEncoderVelocity() {
    return turnMotor.getVelocity();
  }

  public double getDriveEncoderDistance() {
    return driveMotor.getPosition();
  }

  public double getDriveEncoderVelocity() {
    return driveMotor.getVelocity();
  }

  /**
   * Runs the module with the specified voltage while controlling to zero degrees. Must be called
   * periodically.
   */
  public void runCharacterization(Measure<Voltage> volts) {
    // System.out.println(volts.in(Volts));
    setDesiredState(new SwerveModuleState(), false);
    driveMotor.setVoltage(volts.in(Volts));
    // turnMotor.setVoltage(volts.in(Volts));
  }

  @Override
  public void periodic() {}
}

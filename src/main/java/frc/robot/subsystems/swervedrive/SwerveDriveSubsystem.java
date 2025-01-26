// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkSim;
import frc.maps.Constants;
import frc.robot.RobotState;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

/** Class for controlling a swerve drive chassis. Consists of 4 SwerveModules and a gyro. */
public class SwerveDriveSubsystem extends SubsystemBase {

  public static SwerveDriveSubsystem mInstance;

  static final Lock odometryLock = new ReentrantLock();

  private static CCMotorController.MotorFactory defaultMotorFactory = CCSparkSim::new;
  private static SwerveModuleIO.ModuleFactory defaultModuleFactory = SwerveModuleIOSim::new;

  private CCMotorController.MotorFactory motorFactory;
  private SwerveModuleIO.ModuleFactory moduleFactory;

  public SwerveModuleInputsAutoLogged frontRightInputs;
  public SwerveModuleInputsAutoLogged frontLeftInputs;
  public SwerveModuleInputsAutoLogged backRightInputs;
  public SwerveModuleInputsAutoLogged backLeftInputs;

  public SwerveModuleInputsAutoLogged[] swerveModuleInputs;

  public final SwerveModuleIO frontRight;
  public final SwerveModuleIO frontLeft;
  public final SwerveModuleIO backRight;
  public final SwerveModuleIO backLeft;

  // * Must be in the order FR, FL, BR, BL */
  private SwerveModuleIO[] swerveModules;

  public SwerveModuleState[] desiredModuleStates;

  /* PID Controllers */
  public PIDController xPID, yPID;
  public PIDController turnPID;
  public ProfiledPIDController turnPIDProfiled;
  // ProfiledPIDController turnPID;

  /** For old pathplanner */
  public final PPHolonomicDriveController swerveFollower;

  /** Implementation of Singleton Pattern */
  public static SwerveDriveSubsystem getInstance(
      CCMotorController.MotorFactory motorFactory, SwerveModuleIO.ModuleFactory moduleFactory) {
    if (mInstance == null) {
      mInstance = new SwerveDriveSubsystem(motorFactory, moduleFactory);
    }
    return mInstance;
  }

  public static SwerveDriveSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new SwerveDriveSubsystem(defaultMotorFactory, defaultModuleFactory);
    }
    return mInstance;
  }

  /** Constructor for the Swerve Drive Subsystem. */
  private SwerveDriveSubsystem(
      CCMotorController.MotorFactory motorFactory, SwerveModuleIO.ModuleFactory moduleFactory) {
    this.motorFactory = motorFactory;
    this.moduleFactory = moduleFactory;

    frontRight =
        moduleFactory.create(
            motorFactory.create(
                "Front Right Drive",
                "frd",
                Constants.MotorConstants.FRONT_RIGHT_DRIVE,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.FRONT_RIGHT_DRIVE_REVERSE,
                Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
                Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
            motorFactory.create(
                "Front Right Turn",
                "frt",
                Constants.MotorConstants.FRONT_RIGHT_TURN,
                MotorType.kBrushless,
                // Brake
                IdleMode.kCoast,
                Constants.MotorConstants.FRONT_RIGHT_TURN_REVERSE,
                Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
                Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
            Constants.SwerveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER,
            Constants.SwerveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET,
            "Front Right");

    frontLeft =
        moduleFactory.create(
            motorFactory.create(
                "Front Left Drive",
                "fld",
                Constants.MotorConstants.FRONT_LEFT_DRIVE,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.FRONT_LEFT_DRIVE_REVERSE,
                Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
                Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
            motorFactory.create(
                "Front Left Turn",
                "flt",
                Constants.MotorConstants.FRONT_LEFT_TURN,
                MotorType.kBrushless,
                IdleMode.kCoast,
                Constants.MotorConstants.FRONT_LEFT_TURN_REVERSE,
                Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
                Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
            Constants.SwerveConstants.FRONT_LEFT_ABSOLUTE_ENCODER,
            Constants.SwerveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET,
            "Front Left");

    backRight =
        moduleFactory.create(
            motorFactory.create(
                "Back Right Drive",
                "brd",
                Constants.MotorConstants.BACK_RIGHT_DRIVE,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.BACK_RIGHT_DRIVE_REVERSE,
                Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
                Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
            motorFactory.create(
                "Back Right Turn",
                "brt",
                Constants.MotorConstants.BACK_RIGHT_TURN,
                MotorType.kBrushless,
                IdleMode.kCoast,
                Constants.MotorConstants.BACK_RIGHT_TURN_REVERSE,
                Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
                Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
            Constants.SwerveConstants.BACK_RIGHT_ABSOLUTE_ENCODER,
            Constants.SwerveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET,
            "Back Right");

    backLeft =
        moduleFactory.create(
            motorFactory.create(
                "Back Left Drive",
                "bld",
                Constants.MotorConstants.BACK_LEFT_DRIVE,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.BACK_LEFT_DRIVE_REVERSE,
                Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
                Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
            motorFactory.create(
                "Back Left Turn",
                "blt",
                Constants.MotorConstants.BACK_LEFT_TURN,
                MotorType.kBrushless,
                IdleMode.kCoast,
                Constants.MotorConstants.BACK_LEFT_TURN_REVERSE,
                Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
                Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
            Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER,
            Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET,
            "Back Left");

    swerveModules = new SwerveModuleIO[] {frontRight, frontLeft, backRight, backLeft};

    frontRightInputs = new SwerveModuleInputsAutoLogged();
    frontLeftInputs = new SwerveModuleInputsAutoLogged();
    backRightInputs = new SwerveModuleInputsAutoLogged();
    backLeftInputs = new SwerveModuleInputsAutoLogged();

    swerveModuleInputs =
        new SwerveModuleInputsAutoLogged[] {
          frontRightInputs, frontLeftInputs, backRightInputs, backLeftInputs
        };

    RealOdometryThread.getInstance().start();

    desiredModuleStates = new SwerveModuleState[4];

    desiredModuleStates[0] = new SwerveModuleState(0, new Rotation2d());
    desiredModuleStates[1] = new SwerveModuleState(0, new Rotation2d());
    desiredModuleStates[2] = new SwerveModuleState(0, new Rotation2d());
    desiredModuleStates[3] = new SwerveModuleState(0, new Rotation2d());

    xPID = new PIDController(1, .1, 0);
    yPID = new PIDController(1, .1, 0);

    turnPID = new PIDController(.05, .1, 0);

    swerveFollower =
        new PPHolonomicDriveController(
            new com.pathplanner.lib.config.PIDConstants(1, 0, 0),
            new com.pathplanner.lib.config.PIDConstants(1, 0, 0),
            .02);

    turnPIDProfiled =
        new ProfiledPIDController(
            .7,
            0,
            0,
            new Constraints(
                Constants.SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                Constants.SwerveConstants.TURN_RATE_LIMIT));
    turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {

    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].updateInputs(swerveModuleInputs[i]);
      Logger.processInputs("Subsystem/Drive/" + swerveModules[i].getName(), swerveModuleInputs[i]);
    }

    Logger.recordOutput("Actual moduleStates", getCurrentModuleStates());
  }

  /**
   * Creates a new SwerveDrive object. Delays 1 second before setting gyro to 0 to account for gyro
   * calibration time.
   */
  public void getAbsoluteEncoderoffsets() {

    System.out.println("frontRight:" + frontRight.getAbsoluteEncoderRadiansNoOffset());
    System.out.println("frontLeft:" + frontLeft.getAbsoluteEncoderRadiansNoOffset());
    System.out.println("backRight:" + backRight.getAbsoluteEncoderRadiansNoOffset());
    System.out.println("backLeft:" + backLeft.getAbsoluteEncoderRadiansNoOffset());
  }

  /** Sets all 4 modules' drive and turn speeds to 0. */
  public void stopModules() {
    SwerveModuleState[] states =
        new SwerveModuleState[] {
          new SwerveModuleState(0, new Rotation2d(frontRight.getAbsoluteEncoderRadiansOffset())),
          new SwerveModuleState(0, new Rotation2d(frontLeft.getAbsoluteEncoderRadiansOffset())),
          new SwerveModuleState(0, new Rotation2d(backRight.getAbsoluteEncoderRadiansOffset())),
          new SwerveModuleState(0, new Rotation2d(backLeft.getAbsoluteEncoderRadiansOffset()))
        };
    setModuleStates(states);
  }

  /**
   * Sets all 4 modules' drive and turn speeds with the SwerveModuleState format. Everytime you set
   * the robot to move somewhere, it should pass through this method (theoretically).
   *
   * @param desiredStates The array of the states that each module will be set to in the
   *     SwerveModuleState format.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // currentSwerveModuleStates = desiredStates;
    // desiredModuleStates = desiredStates;

    boolean openLoop = false;
    // SwerveDriveKinematics.desaturateWheelSpeeds(
    //     desiredStates, Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL);
    // Logger.recordOutput("SwerveModuleStates/SetpointsOptimized", desiredStates);

    Logger.recordOutput("desired moduleStates", desiredStates);
    frontRight.setDesiredState(desiredStates[0], openLoop);
    frontLeft.setDesiredState(desiredStates[1], openLoop);
    backRight.setDesiredState(desiredStates[2], openLoop);
    backLeft.setDesiredState(desiredStates[3], openLoop);
    Logger.recordOutput("postChange desired moduleStates", desiredStates);
  }

  /* Returns the actual moduleStates */
  public SwerveModuleState[] getCurrentModuleStates() {
    SwerveModuleState[] states =
        new SwerveModuleState[] {
          frontRight.getState(), frontLeft.getState(), backRight.getState(), backLeft.getState()
        };
    return states;
  }

  /*
   * Used for Autobuilder in AutonomousScheme.java
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {

    return ChassisSpeeds.fromRobotRelativeSpeeds(
        Constants.SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getCurrentModuleStates()),
        RobotState.getInstance().getRotation2d());
  }

  public ChassisSpeeds getFieldVelocity() {
    // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
    // but not the reverse.  However, because this transform is a simple rotation, negating the
    // angle
    // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        Constants.SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getCurrentModuleStates()),
        RobotState.getInstance().poseEstimator.getEstimatedPosition().getRotation());
    // RobotState.getInstance().getRotation2d());
  }

  /*
   * Used for Autobuilder in AutonomousScheme.java
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    Logger.recordOutput("desired Chassis Speeds", chassisSpeeds);

    SwerveModuleState[] moduleStates =
        Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(moduleStates);
  }

  private final VelocityUnit<VoltageUnit> VoltsPerSecond = Volts.per(Second);

  SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              VoltsPerSecond.of(1),
              Volts.of(3),
              Seconds.of(3),
              (state) ->
                  org.littletonrobotics.junction.Logger.recordOutput(
                      "SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> setDriveVoltages(voltage),
              null, // No log consumer, since data is recorded by URCL
              this));

  /* SysID Factory Methods */
  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the quasistatic characterization test
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the dynamic characterization test
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /**
   * Used only in Characterizing. Don't touch this. Sets the provided voltages and locks the wheels
   * to 0 radians.
   *
   * @param volts
   */
  public void setDriveVoltages(Voltage volts) {
    for (SwerveModuleIO s : swerveModules) {
      s.setTurnPosition(() -> 0);
      s.setDriveVoltage(volts.in(Volts));
    }
  }

  public void spinMotor(double speed) {
    frontRight.setDriveVelocity(speed);
    frontRight.setTurnPosition(() -> speed);
  }

  public void test(double driveSpeed, double turnSpeed) {
    backRight.driveAndTurn(driveSpeed, turnSpeed);
    // backRight.printEncoders();
  }

  public void printWorld() {
    System.out.println("Hello World!");
  }

  public void setRawDriveVolts(double volt) {
    frontRight.setDriveVoltage(volt);
    frontLeft.setDriveVoltage(volt);
    backRight.setDriveVoltage(volt);
    backLeft.setDriveVoltage(volt);

    frontRight.setTurnPosition(() -> Math.PI / 2);
    frontLeft.setTurnPosition(() -> Math.PI / 2);
    backRight.setTurnPosition(() -> Math.PI / 2);
    backLeft.setTurnPosition(() -> Math.PI / 2);
  }

  public Command halt() {
    return Commands.runOnce(() -> {}, this);
  }

  public Command resetTurnEncoders() {
    return new InstantCommand(
        () -> {
          frontRight.resetTurnEncoder();
          frontLeft.resetTurnEncoder();
          backRight.resetTurnEncoder();
          backLeft.resetTurnEncoder();
        });
  }

  public void setspeeds(double speed) {
    frontRight.setDriveVelocity(speed);
    frontLeft.setDriveVelocity(speed);
    backRight.setDriveVelocity(speed);
    backLeft.setDriveVelocity(speed);
  }

  public static double getAdjustedYaw(double angle) {
    while (angle > Math.PI) {
      angle -= 2 * Math.PI;
    }
    while (angle < -Math.PI) {
      angle += 2 * Math.PI;
    }
    return angle;
  }
}

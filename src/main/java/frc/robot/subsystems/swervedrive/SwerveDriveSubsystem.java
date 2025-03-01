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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.util.maps.Constants;
import frc.util.motorcontroller.CCMotorController;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

/** Class for controlling a swerve drive chassis. Consists of 4 SwerveModules and a gyro. */
public class SwerveDriveSubsystem extends SubsystemBase {

  public static final Lock odometryLock = new ReentrantLock();

  public final SwerveModuleIO frontRight;
  public final SwerveModuleIO frontLeft;
  public final SwerveModuleIO backRight;
  public final SwerveModuleIO backLeft;

  private final SwerveModuleIO[] swerveModules;

  private final SysIdRoutine sysIdRoutine;

  // * Must be in the order FR, FL, BR, BL */
  public final SwerveModuleInputsAutoLogged frontRightInputs = new SwerveModuleInputsAutoLogged();
  public final SwerveModuleInputsAutoLogged frontLeftInputs = new SwerveModuleInputsAutoLogged();
  public final SwerveModuleInputsAutoLogged backRightInputs = new SwerveModuleInputsAutoLogged();
  public final SwerveModuleInputsAutoLogged backLeftInputs = new SwerveModuleInputsAutoLogged();

  public final SwerveModuleInputsAutoLogged[] swerveModuleInputs;

  public SwerveModuleState[] desiredModuleStates;

  /* PID Controllers */
  public PIDController xPID, yPID;
  public PIDController turnPID;
  public ProfiledPIDController turnPIDProfiled;

  public PIDController translationPID;
  public PIDController rotationPID;

  /** For old pathplanner */
  public final PPHolonomicDriveController swerveFollower;

  /** Constructor for the Swerve Drive Subsystem. */
  public SwerveDriveSubsystem(
      CCMotorController.MotorFactory motorFactory, SwerveModuleIO.ModuleFactory moduleFactory) {

    frontRight =
        moduleFactory.create(
            motorFactory.create(
                "Front Right Drive",
                "frd",
                Constants.MotorConstants.FRONT_RIGHT_DRIVE,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.FRONT_RIGHT_DRIVE_REVERSE,
                // Constants.ConversionConstants.ONE_METER_PER_MOTOR_ROTATIONS,
                Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
                Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
            motorFactory.create(
                "Front Right Turn",
                "frt",
                Constants.MotorConstants.FRONT_RIGHT_TURN,
                MotorType.kBrushless,
                IdleMode.kBrake,
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
                // Constants.ConversionConstants.ONE_METER_PER_MOTOR_ROTATIONS,
                Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
                Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
            motorFactory.create(
                "Front Left Turn",
                "flt",
                Constants.MotorConstants.FRONT_LEFT_TURN,
                MotorType.kBrushless,
                IdleMode.kBrake,
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
                // Constants.ConversionConstants.ONE_METER_PER_MOTOR_ROTATIONS,
                Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
                Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
            motorFactory.create(
                "Back Right Turn",
                "brt",
                Constants.MotorConstants.BACK_RIGHT_TURN,
                MotorType.kBrushless,
                IdleMode.kBrake,
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
                // Constants.ConversionConstants.ONE_METER_PER_MOTOR_ROTATIONS,
                Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
                Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
            motorFactory.create(
                "Back Left Turn",
                "blt",
                Constants.MotorConstants.BACK_LEFT_TURN,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.BACK_LEFT_TURN_REVERSE,
                Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
                Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
            Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER,
            Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET,
            "Back Left");

    swerveModules = new SwerveModuleIO[] {frontRight, frontLeft, backRight, backLeft};

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1),
                Volts.of(5),
                Seconds.of(5),
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> setDriveVoltages(voltage),
                null, // No log consumer, since data is recorded by URCL
                this));

    swerveModuleInputs =
        new SwerveModuleInputsAutoLogged[] {
          frontRightInputs, frontLeftInputs, backRightInputs, backLeftInputs
        };

    desiredModuleStates = new SwerveModuleState[4];

    desiredModuleStates[0] = new SwerveModuleState(0, new Rotation2d());
    desiredModuleStates[1] = new SwerveModuleState(0, new Rotation2d());
    desiredModuleStates[2] = new SwerveModuleState(0, new Rotation2d());
    desiredModuleStates[3] = new SwerveModuleState(0, new Rotation2d());

    xPID = new PIDController(.5, 0, .1);
    yPID = new PIDController(.5, 0, .1);

    turnPID = new PIDController(.05, .1, 0);

    translationPID = new PIDController(2, 0, 0);
    rotationPID = new PIDController(5, 0, 0);
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putData("Swerve Translation PID", translationPID);
    SmartDashboard.putData("Swerve Rotation PID", rotationPID);

    swerveFollower =
        new PPHolonomicDriveController(
            new com.pathplanner.lib.config.PIDConstants(2, 0, 0),
            new com.pathplanner.lib.config.PIDConstants(2, 0, 0),
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
      Logger.recordOutput(
          swerveModules[i].getName() + "/getTurnPosition", swerveModules[i].getTurnPosition());
      Logger.processInputs("Subsystem/Drive/" + swerveModules[i].getName(), swerveModuleInputs[i]);
    }

    // getAbsoluteEncoderoffsets();
    Logger.recordOutput("Swerve Drive Outputs/Actual Module States", getCurrentModuleStates());
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

    Logger.recordOutput("Swerve Drive Outputs/Desired Module States", desiredStates);
    frontRight.setDesiredState(desiredStates[0], openLoop);
    frontLeft.setDesiredState(desiredStates[1], openLoop);
    backRight.setDesiredState(desiredStates[2], openLoop);
    backLeft.setDesiredState(desiredStates[3], openLoop);
    Logger.recordOutput("Swerve Drive Outputs/Processed Desired Module States", desiredStates);
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
    Logger.recordOutput("Swerve Drive Outputs/Desired Chassis Speeds", chassisSpeeds);

    SwerveModuleState[] moduleStates =
        Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(moduleStates);
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
}

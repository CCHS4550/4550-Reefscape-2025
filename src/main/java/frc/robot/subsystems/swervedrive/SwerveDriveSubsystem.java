// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
import frc.maps.Constants.SwerveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveModuleInputsAutoLogged;


import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Class for controlling a swerve drive chassis. Consists of 4 SwerveModules and a gyro. */
public class SwerveDriveSubsystem extends SubsystemBase {

  public static SwerveDriveSubsystem mInstance;


  private static CCMotorController.MotorFactory defaultMotorFactory = CCSparkMax::new;
  private static SwerveModuleIO.ModuleFactory defaultModuleFactory = SwerveModuleIOHardware::new;


  private CCMotorController.MotorFactory motorFactory;
  private SwerveModuleIO.ModuleFactory moduleFactory;


  SwerveModuleInputsAutoLogged frontRightInputs = new SwerveModuleInputsAutoLogged();
  SwerveModuleInputsAutoLogged frontLeftInputs = new SwerveModuleInputsAutoLogged();
  SwerveModuleInputsAutoLogged backRightInputs = new SwerveModuleInputsAutoLogged();
  SwerveModuleInputsAutoLogged backLeftInputs = new SwerveModuleInputsAutoLogged();

  public final SwerveModuleIO frontRight =
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
              IdleMode.kBrake,
              Constants.MotorConstants.FRONT_RIGHT_TURN_REVERSE,
              Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
              Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
          Constants.SwerveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER,
          Constants.SwerveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET,
          "Front Right");

  public final SwerveModuleIO frontLeft =
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
              IdleMode.kBrake,
              Constants.MotorConstants.FRONT_LEFT_TURN_REVERSE,
              Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
              Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
          Constants.SwerveConstants.FRONT_LEFT_ABSOLUTE_ENCODER,
          Constants.SwerveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET,
          "Front Left");

  public final SwerveModuleIO backRight =
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
              IdleMode.kBrake,
              Constants.MotorConstants.BACK_RIGHT_TURN_REVERSE,
              Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
              Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
          Constants.SwerveConstants.BACK_RIGHT_ABSOLUTE_ENCODER,
          Constants.SwerveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET,
          "Back Right");

  public final SwerveModuleIO backLeft =
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
              IdleMode.kBrake,
              Constants.MotorConstants.BACK_LEFT_TURN_REVERSE,
              Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
              Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
          Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER,
          Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET,
          "Back Left");

  // * Must be in the order FR, FL, BR, BL */
  private SwerveModuleIO[] swerveModules =
      new SwerveModuleIO[] {frontRight, frontLeft, backRight, backLeft};

  public SwerveModuleState[] desiredModuleStates;

  /** Module positions used for odometry */
  public SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

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
    
    swerveModulePositions[0] =
        new SwerveModulePosition(0, new Rotation2d(frontRight.getAbsoluteEncoderRadiansOffset()));
    swerveModulePositions[1] =
        new SwerveModulePosition(0, new Rotation2d(frontLeft.getAbsoluteEncoderRadiansOffset()));
    swerveModulePositions[2] =
        new SwerveModulePosition(0, new Rotation2d(backRight.getAbsoluteEncoderRadiansOffset()));
    swerveModulePositions[3] =
        new SwerveModulePosition(0, new Rotation2d(backLeft.getAbsoluteEncoderRadiansOffset()));

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
    // swerveFollower = new PPHolonomicDriveController(xPID, yPID, turnPID);

    // swerveFollower.setEnabled(true);
    // xPID = new PIDController(1, 0, 0);
    // yPID = new PIDController(1, 0, 0);

    // *TODO: Possibly research profiled PID
    // turnPID = new ProfiledPIDController(0.5, 0, 0,
    // RobotMap.thetaControllConstraints);

    turnPIDProfiled =
        new ProfiledPIDController(
            .7,
            0,
            0,
            new Constraints(
                Constants.SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                Constants.SwerveConstants.TURN_RATE_LIMIT));
    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    RobotState.getInstance().moduleEncodersInit();
                
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

  /** Returns the nearest speaker pose for for alliance color */
  @Override
  public void periodic() {

    // getAbsoluteEncoderoffsets();
    Logger.recordOutput("Actual moduleStates", getCurrentModuleStates());

    RobotState.getInstance().updateModuleEncoders();
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
    desiredModuleStates = desiredStates;

    boolean openLoop = false;
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL);
    Logger.recordOutput("SwerveModuleStates/SetpointsOptimized", desiredStates);

    frontRight.setDesiredState(desiredStates[0], openLoop);
    frontLeft.setDesiredState(desiredStates[1], openLoop);
    backRight.setDesiredState(desiredStates[2], openLoop);
    backLeft.setDesiredState(desiredStates[3], openLoop);

    Logger.recordOutput("Desired moduleStates", desiredStates);
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

    SwerveModuleState[] moduleStates =
        Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    // Logger.recordOutput("Autonomous Set moduleStates", moduleStates);
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



    public static double getAdjustedYaw(double angle){
        while (angle > Math.PI){
            angle -= 2*Math.PI;

        }
        while (angle < -Math.PI){
            angle += 2*Math.PI;
        }
        return angle;
    }

  
}

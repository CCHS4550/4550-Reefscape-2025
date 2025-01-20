// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.util.ChoreoAllianceFlipUtil;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.helpers.vision.PhotonVision;
import frc.maps.Constants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.RealOdometryThread;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

/** RobotState is used to retrieve information about the robot's state in other classes. */
public class RobotState {

  public static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  SwerveDriveSubsystem swerve;
  AlgaeSubsystem algae;
  ArmSubsystem arm;
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;
  WristSubsystem wrist;

  /** Module positions used for odometry */
  public SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

  // Initialize gyro
  public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private final Queue<Double> gyroContainer;
  public Rotation2d[] gyroAngle = new Rotation2d[] {};

  private RobotState() {

    new Thread(
            () -> {
              try {
                Thread.sleep(1000);
                zeroHeading();
              } catch (Exception e) {
              }
            })
        .start();

    gyroContainer =
        RealOdometryThread.getInstance().registerInput(() -> getRotation2d().getRadians());
  }

  public void robotStateInit(
      SwerveDriveSubsystem swerve,
      AlgaeSubsystem algae,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake,
      WristSubsystem wrist) {
    this.swerve = swerve;
    this.algae = algae;
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;
    this.wrist = wrist;

    swerveModulePositions[0] =
        new SwerveModulePosition(
            0, new Rotation2d(swerve.frontRight.getAbsoluteEncoderRadiansOffset()));
    swerveModulePositions[1] =
        new SwerveModulePosition(
            0, new Rotation2d(swerve.frontLeft.getAbsoluteEncoderRadiansOffset()));
    swerveModulePositions[2] =
        new SwerveModulePosition(
            0, new Rotation2d(swerve.backRight.getAbsoluteEncoderRadiansOffset()));
    swerveModulePositions[3] =
        new SwerveModulePosition(
            0, new Rotation2d(swerve.backLeft.getAbsoluteEncoderRadiansOffset()));
  }

  public final Field2d gameField = new Field2d();

  /** Technically pose ESTIMATES */
  public Pose2d lastPose = new Pose2d();

  public Pose2d currentPose = new Pose2d();

  public SwerveDrivePoseEstimator poseEstimator;

  public final frc.helpers.vision.VisionIOInputsAutoLogged visionInputs =
      new frc.helpers.vision.VisionIOInputsAutoLogged();

  public synchronized void poseInit() {

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.DRIVE_KINEMATICS,
            getRotation2d(),
            swerveModulePositions,
            new Pose2d(0, 0, new Rotation2d(0)));
  }

  public synchronized void updateOdometryPose() {

    gyroAngle =
        gyroContainer.stream()
            .map((Double value) -> new Rotation2d(value))
            .toArray(Rotation2d[]::new);
    gyroContainer.clear();

    gameField.setRobotPose(getPose());

    lastPose = currentPose;

    double sampleCount = swerve.swerveModuleInputs[0].odometryTimestamps.length;
    double[] sampleTimestamps = swerve.swerveModuleInputs[0].odometryTimestamps;

    /** Update the SwerveDrivePoseEstimator with the Drivetrain encoders and such */
    for (int i = 0; i < sampleCount; i++) {

      poseEstimator.updateWithTime(sampleTimestamps[i], gyroAngle[i], getModulePositionArray()[i]);
    }

    currentPose = getPose();
    Logger.recordOutput("Estimated Pose", getPose());
    Logger.recordOutput("Estimated Angle", getPose().getRotation().getDegrees());
  }

  public synchronized void updateVisionPose() {
    /** Update the visionData to what the camera sees. */
    if (Robot.isReal()) {
      PhotonVision.getInstance().updateInputs(visionInputs, getPose());

      for (int i = 0; i < visionInputs.poseEstimates.length; i++) {
        /** Add the Photonvision pose estimates */
        poseEstimator.addVisionMeasurement(visionInputs.poseEstimates[i], visionInputs.timestamp);
      }
    } else if (Robot.isSimulation()) {
      PhotonVision.getInstance().visionSim.update(getPose());
    }
  }

  public synchronized void dashboardInit() {

    /* Put the Command Scheduler on SmartDashboard */
    SmartDashboard.putData(CommandScheduler.getInstance());

    /* Put all the subsystems on ShuffleBoard in their own "Subsystems" tab. */
    Shuffleboard.getTab("Subsystems").add("Swerve Drive", swerve);
    Shuffleboard.getTab("Subsystems").add("Algae", algae);
    Shuffleboard.getTab("Subsystems").add("Arm", arm);
    Shuffleboard.getTab("Subsystems").add("Elevator", elevator);
    Shuffleboard.getTab("Subsystems").add("Wrist", wrist);
    Shuffleboard.getTab("Subsystems").add("Intake", intake);

    /* Put the Pose Estimators on Dashboards */
    SmartDashboard.putData("Field", gameField);
  }

  public synchronized void updateDashboard() {

    SmartDashboard.putNumber("Robot X Position", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Robot Y Position", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber(
        "Robot Rads Angle", poseEstimator.getEstimatedPosition().getRotation().getRadians());
    SmartDashboard.putNumber(
        "Robot Degrees Angle", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }

  public GenericEntry yActual, yGoal, xActual, xGoal, barrelActual, barrelGoal;
  // ** NetworkTableEntry for the encoders of the turn motors */
  private GenericEntry abs_Enc_FR_Offset_Entry,
      abs_Enc_FL_Offset_Entry,
      abs_Enc_BR_Offset_Entry,
      abs_Enc_BL_Offset_Entry;
  private GenericEntry abs_Enc_FR_Raw_Entry,
      abs_Enc_FL_Raw_Entry,
      abs_Enc_BR_Raw_Entry,
      abs_Enc_BL_Raw_Entry;
  private GenericEntry enc_FR_pos_Entry, enc_FL_pos_Entry, enc_BR_pos_Entry, enc_BL_pos_Entry;

  // ShuffleBoardLayouts for putting encoders onto the board
  private ShuffleboardLayout absolute_encoders_offset_list =
      Shuffleboard.getTab("Encoders")
          .getLayout("Absolute Encoders Offset", BuiltInLayouts.kGrid)
          .withSize(2, 2);

  private ShuffleboardLayout absolute_encoders_no_offset_list =
      Shuffleboard.getTab("Encoders")
          .getLayout("Absolute Encoders No Offset", BuiltInLayouts.kGrid)
          .withSize(2, 2);
  private ShuffleboardLayout turn_encoders_positions =
      Shuffleboard.getTab("Encoders")
          .getLayout("Turn Encoders Position(Rad)", BuiltInLayouts.kGrid)
          .withSize(2, 2);

  public synchronized void moduleEncodersInit() {

    abs_Enc_FR_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(swerve.frontRight.getName(), swerve.frontRight.getAbsoluteEncoderRadiansOffset())
            .getEntry();
    abs_Enc_FL_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(swerve.frontLeft.getName(), swerve.frontLeft.getAbsoluteEncoderRadiansOffset())
            .getEntry();
    abs_Enc_BR_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(swerve.backRight.getName(), swerve.backRight.getAbsoluteEncoderRadiansOffset())
            .getEntry();
    abs_Enc_BL_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(swerve.backLeft.getName(), swerve.backLeft.getAbsoluteEncoderRadiansOffset())
            .getEntry();

    enc_FR_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(swerve.frontRight.getName(), swerve.frontRight.getTurnPosition())
            .getEntry();
    enc_FL_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(swerve.frontLeft.getName(), swerve.frontLeft.getTurnPosition())
            .getEntry();
    enc_BR_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(swerve.backRight.getName(), swerve.backRight.getTurnPosition())
            .getEntry();
    enc_BL_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(swerve.backLeft.getName(), swerve.backLeft.getTurnPosition())
            .getEntry();

    abs_Enc_FR_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(swerve.frontRight.getName(), swerve.frontRight.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
    abs_Enc_FL_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(swerve.frontLeft.getName(), swerve.frontLeft.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
    abs_Enc_BR_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(swerve.backRight.getName(), swerve.backRight.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
    abs_Enc_BL_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(swerve.backLeft.getName(), swerve.backLeft.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
  }

  public synchronized void updateModuleEncoders() {
    abs_Enc_FR_Offset_Entry.setDouble(swerve.frontRight.getAbsoluteEncoderRadiansOffset());
    abs_Enc_FL_Offset_Entry.setDouble(swerve.frontLeft.getAbsoluteEncoderRadiansOffset());
    abs_Enc_BR_Offset_Entry.setDouble(swerve.backRight.getAbsoluteEncoderRadiansOffset());
    abs_Enc_BL_Offset_Entry.setDouble(swerve.backLeft.getAbsoluteEncoderRadiansOffset());

    abs_Enc_FR_Raw_Entry.setDouble(swerve.frontRight.getAbsoluteEncoderRadiansNoOffset());
    abs_Enc_FL_Raw_Entry.setDouble(swerve.frontLeft.getAbsoluteEncoderRadiansNoOffset());
    abs_Enc_BR_Raw_Entry.setDouble(swerve.backRight.getAbsoluteEncoderRadiansNoOffset());
    abs_Enc_BL_Raw_Entry.setDouble(swerve.backLeft.getAbsoluteEncoderRadiansNoOffset());

    enc_FR_pos_Entry.setDouble(swerve.frontRight.getTurnPosition());
    enc_FL_pos_Entry.setDouble(swerve.frontLeft.getTurnPosition());
    enc_BR_pos_Entry.setDouble(swerve.backRight.getTurnPosition());
    enc_BL_pos_Entry.setDouble(swerve.backLeft.getTurnPosition());
  }

  public synchronized SwerveModulePosition[][] getModulePositionArray() {

    int sampleCount = swerve.swerveModuleInputs[0].odometryTimestamps.length;
    SwerveModulePosition[][] positions = new SwerveModulePosition[sampleCount][4];

    for (int i = 0; i < sampleCount; i++) {
      positions[i][0] =
          new SwerveModulePosition(
              swerve.swerveModuleInputs[0].odometryDrivePositionsMeters[i],
              swerve.swerveModuleInputs[0].odometryTurnPositions[i]);
      positions[i][1] =
          new SwerveModulePosition(
              swerve.swerveModuleInputs[1].odometryDrivePositionsMeters[i],
              swerve.swerveModuleInputs[1].odometryTurnPositions[i]);
      positions[i][2] =
          new SwerveModulePosition(
              swerve.swerveModuleInputs[2].odometryDrivePositionsMeters[i],
              swerve.swerveModuleInputs[2].odometryTurnPositions[i]);
      positions[i][3] =
          new SwerveModulePosition(
              swerve.swerveModuleInputs[3].odometryDrivePositionsMeters[i],
              swerve.swerveModuleInputs[3].odometryTurnPositions[i]);
    }

    return positions;
  }

  public synchronized void updateModulePositions() {
    swerveModulePositions[0] =
        new SwerveModulePosition(
            swerve.frontRight.getDrivePosition(),
            new Rotation2d(swerve.frontRight.getTurnPosition()));
    swerveModulePositions[1] =
        new SwerveModulePosition(
            swerve.frontLeft.getDrivePosition(),
            new Rotation2d(swerve.frontLeft.getTurnPosition()));
    swerveModulePositions[2] =
        new SwerveModulePosition(
            swerve.backRight.getDrivePosition(),
            new Rotation2d(swerve.backRight.getTurnPosition()));
    swerveModulePositions[3] =
        new SwerveModulePosition(
            swerve.backLeft.getDrivePosition(), new Rotation2d(swerve.backLeft.getTurnPosition()));
  }

  public synchronized double[] getAverageTimestampArray() {

    int inputQuantity = swerve.swerveModuleInputs[0].odometryTimestamps.length;

    double[] averages = new double[inputQuantity];

    for (int i = 0; i < inputQuantity; i++) {
      double average =
          (swerve.swerveModuleInputs[0].odometryTimestamps[i]
                  + swerve.swerveModuleInputs[1].odometryTimestamps[i]
                  + swerve.swerveModuleInputs[2].odometryTimestamps[i]
                  + swerve.swerveModuleInputs[3].odometryTimestamps[i])
              / 4;

      averages[i] = average;
    }

    return averages;
  }

  /** Pose Helper Methods */

  /**
   * Gets the position of the robot in Pose2d format. Uses odometer reading. Includes the x, y, and
   * theta values of the robot.
   *
   * @return The Pose2d of the robot.
   */
  public synchronized Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // /** If Using REFERENCE POSE STRAT */
  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
  //   photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
  //   return photonPoseEstimator.update();
  // }

  public synchronized void printPos2d() {
    System.out.println(poseEstimator.getEstimatedPosition());
  }

  // Returns the estimated transformation over the next tick (The change in
  // position)
  // private Transform2d getTickFutureTransform() {
  //   return new Transform2d(
  //       new Translation2d(
  //           swerve.getFieldVelocity().vxMetersPerSecond * 0.02,
  //           swerve.getFieldVelocity().vyMetersPerSecond * 0.02),
  //       new Rotation2d(0.0));
  // }

  // // Returns the estimated robot position a tick from the current time (Theoretically?)
  // private Pose2d getFutureTickPose() {
  //   return getPose().plus(getTickFutureTransform().inverse());
  // }

  /** Odometry */

  /**
   * Resets the odometer readings using the gyro, SwerveModulePositions (defined in constructor),
   * and Pose2d. Also used in AutonomousScheme.java
   */
  public synchronized void setOdometry() {
    poseEstimator.resetPosition(getRotation2d(), swerveModulePositions, getPose());
  }

  /**
   * Resets the odometer readings using the gyro, SwerveModulePositions (defined in constructor),
   * and Pose2d. Also used in AutonomousScheme.java
   *
   * @param pos the Pose2d to set the odometry
   */
  public synchronized void setOdometry(Pose2d pos) {
    poseEstimator.resetPosition(
        getRotation2d(),
        // pos.getRotation(),
        swerveModulePositions,
        pos);
  }

  public synchronized void setOdometryAllianceFlip(Pose2d pos) {
    if (Constants.isBlue()) return;

    poseEstimator.resetPosition(
        ChoreoAllianceFlipUtil.flip(getRotation2d()),
        swerveModulePositions,
        ChoreoAllianceFlipUtil.flip(pos));
    return;
  }

  /** Gyroscope Methods (NavX) */
  public synchronized void zeroHeading() {
    gyro.reset();
    setOdometry(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(0)));
  }

  public synchronized double getPitch() {
    return gyro.getPitch() - 1.14;
  }

  public synchronized double getRoll() {
    return gyro.getRoll();
  }

  /**
   * Method to get the facing direction of the gyro.
   *
   * @return The facing direction of the gyro, between -360 and 360 degrees.
   */
  public synchronized double getHeading() {
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public synchronized Rotation2d getPoseRotation2d() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  public synchronized double getPoseAngleDegrees() {
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  public synchronized double getPoseAngleRadians() {
    return poseEstimator.getEstimatedPosition().getRotation().getRadians();
  }

  /**
   * Specifically for the poseEstimator. Anywhere else, get the poseEstimator's estimated heading,
   * or it'll mess things up.
   *
   * @return
   */
  public synchronized Rotation2d getRotation2d() {

    Logger.recordOutput("Gyro Rotation2d", gyro.getRotation2d());
    return gyro.getRotation2d();
    // .plus(Rotation2d.fromRadians(Math.PI));
  }

  public synchronized void setRotation2d(Rotation2d rotation2d) {
    setOdometry(new Pose2d(getPose().getTranslation(), rotation2d));
  }
}

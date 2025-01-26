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
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.helpers.vision.VisionIO;
import frc.maps.Constants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.RealOdometryThread;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
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

  VisionIO vision;

  /** Module positions used for odometry */
  public SwerveModulePosition[] previousSwerveModulePositions = new SwerveModulePosition[4];

  public SwerveModulePosition[] currentSwerveModulePositions = new SwerveModulePosition[4];

  public SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

  public int sampleCount;
  public double[] sampleTimestamps;
  public SwerveModulePosition[][] swerveModulePositionsArray;

  // Initialize gyro
  public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private final Queue<Double> gyroContainer;
  public Rotation2d[] gyroAngle = new Rotation2d[] {};
  public List<Rotation2d> gyroAnglePlusDeltas = new ArrayList<>();
  public double gyroAngleSim = 0;

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
      WristSubsystem wrist,
      VisionIO vision) {
    this.swerve = swerve;
    this.algae = algae;
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;
    this.wrist = wrist;

    this.vision = vision;

    swerveModulePositions[0] =
        new SwerveModulePosition(0, new Rotation2d(swerve.frontRight.getTurnPosition()));
    swerveModulePositions[1] =
        new SwerveModulePosition(0, new Rotation2d(swerve.frontLeft.getTurnPosition()));
    swerveModulePositions[2] =
        new SwerveModulePosition(0, new Rotation2d(swerve.backRight.getTurnPosition()));
    swerveModulePositions[3] =
        new SwerveModulePosition(0, new Rotation2d(swerve.backLeft.getTurnPosition()));

    previousSwerveModulePositions[0] = new SwerveModulePosition();
    previousSwerveModulePositions[1] = new SwerveModulePosition();
    previousSwerveModulePositions[2] = new SwerveModulePosition();
    previousSwerveModulePositions[3] = new SwerveModulePosition();

    currentSwerveModulePositions[0] = new SwerveModulePosition();
    currentSwerveModulePositions[1] = new SwerveModulePosition();
    currentSwerveModulePositions[2] = new SwerveModulePosition();
    currentSwerveModulePositions[3] = new SwerveModulePosition();
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
            new Pose2d(0, 0, new Rotation2d()));
  }

  public synchronized void updateOdometryPose() {

    /** Send the high frequency odometry to an array */
    gyroAngle =
        gyroContainer.stream()
            .map((Double value) -> new Rotation2d(value))
            .toArray(Rotation2d[]::new);
    gyroContainer.clear();

    /**
     * Only just found out this method is basically useless unless your gyro has disconnected. Too
     * tired to explain it rn.
     */
    gameField.setRobotPose(getPose());

    lastPose = currentPose;

    if (!gyro.isConnected()) {
      addModuleDeltas();
      gyroAngle = gyroAnglePlusDeltas.toArray(new Rotation2d[0]);
    }
    if (swerve.swerveModuleInputs[0].odometryTimestamps.length > 0 && gyro.isConnected()) {
      sampleCount = swerve.swerveModuleInputs[0].odometryTimestamps.length;
      sampleTimestamps = swerve.swerveModuleInputs[0].odometryTimestamps;
      swerveModulePositionsArray = getModulePositionArray();

      /** Update the SwerveDrivePoseEstimator with the Drivetrain encoders and such */
      for (int i = 0; i < sampleCount; i++) {

        // System.out.println("sampleTimeStamps size" + sampleTimestamps.length);
        // System.out.println("gyroAngle size" + gyroAngle.length);
        // System.out.println("swerveModulePositionsArray size" +
        // swerveModulePositionsArray.length);

        poseEstimator.updateWithTime(
            getSampleTimestamp(i), getGyroAngle(i), getSwerveModulePositionsArray(i));
        Logger.recordOutput("High Frequency Gyro Angle", getGyroAngle(i));
        Logger.recordOutput("High Frequency Odometry Positions", getSwerveModulePositionsArray(i));
      }

    } else {
      poseEstimator.updateWithTime(
          Timer.getFPGATimestamp() / 1e6, getRotation2d(), swerveModulePositions);
    }

    currentPose = getPose();
    Logger.recordOutput("SwerveDrivePoseEstimator/Estimated Pose", getPose());
    Logger.recordOutput(
        "SwerveDrivePoseEstimator/Estimated Angle Degrees", getPose().getRotation().getDegrees());
    Logger.recordOutput(
        "SwerveDrivePoseEstimator/Estimated Angle Radians", getPose().getRotation().getRadians());
  }

  public synchronized void updateVisionPose() {
    /** Update the visionData to what the camera sees. */
    vision.updateInputs(visionInputs);

    for (int i = 0; i < visionInputs.poseEstimates.length; i++) {
      /** Add the Photonvision pose estimates */
      poseEstimator.addVisionMeasurement(
          visionInputs.poseEstimates[i], visionInputs.timestampArray[i]);
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

    int[] numbers = {
      swerve.swerveModuleInputs[0].odometryDrivePositionsMeters.length,
      swerve.swerveModuleInputs[1].odometryDrivePositionsMeters.length,
      swerve.swerveModuleInputs[2].odometryDrivePositionsMeters.length,
      swerve.swerveModuleInputs[3].odometryDrivePositionsMeters.length,
      swerve.swerveModuleInputs[0].odometryTurnPositions.length,
      swerve.swerveModuleInputs[1].odometryTurnPositions.length,
      swerve.swerveModuleInputs[2].odometryTurnPositions.length,
      swerve.swerveModuleInputs[3].odometryTurnPositions.length,
    };

    int min = Arrays.stream(numbers).min().getAsInt();

    SwerveModulePosition[][] positions = new SwerveModulePosition[min][4];

    for (int i = 0; i < min; i++) {
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

    if (min != 0) swerveModulePositions = positions[min - 1];

    return positions;
  }

  public synchronized void addModuleDeltas() {
    gyroAnglePlusDeltas.clear();

    SwerveModulePosition[] previousPositions = swerveModulePositions;

    for (int i = 0; i < (gyroAngle.length); i++) {
      SwerveModulePosition[] samplePositions = getModulePositionArray()[i];

      Twist2d deltas =
          Constants.SwerveConstants.DRIVE_KINEMATICS.toTwist2d(previousPositions, samplePositions);

      gyroAnglePlusDeltas.add(gyroAngle[i].plus(Rotation2d.fromRadians(deltas.dtheta)));

      previousPositions = samplePositions;
    }
  }

  public synchronized SwerveModulePosition[] updateModulePositions() {

    // previousSwerveModulePositions = swerveModulePositions;

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

    Logger.recordOutput("currentModulePositions", swerveModulePositions);
    Logger.recordOutput("previousModulePositions", previousSwerveModulePositions);

    return swerveModulePositions;
  }

  public SwerveModulePosition[] getSwerveModulePositionsArray(int index) {
    if (index > swerveModulePositionsArray.length - 1) {

      return swerveModulePositionsArray[swerveModulePositionsArray.length - 1];
    }
    return swerveModulePositionsArray[index];
  }

  public double getSampleTimestamp(int index) {
    if (index > sampleTimestamps.length - 1) {

      return sampleTimestamps[sampleTimestamps.length - 1];
    }
    return sampleTimestamps[index];
  }

  public Rotation2d getGyroAngle(int index) {
    if (index > gyroAngle.length - 1) {
      return gyroAngle[gyroAngle.length - 1];
    }
    return gyroAngle[index];
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

    // currentSwerveModulePositions = updateModulePositions();

    if (RobotBase.isSimulation()) {
      /**
       * This code in this if block is from Mechanical Advantage's Spark Swerve Project. IDK why it
       * works and my version didn't, but it does.
       */
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = swerveModulePositions[moduleIndex];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - previousSwerveModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        previousSwerveModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      gyroAngleSim += Constants.SwerveConstants.DRIVE_KINEMATICS.toTwist2d(moduleDeltas).dtheta;

      Logger.recordOutput("gyroAngleSim", gyroAngleSim);

      previousSwerveModulePositions = currentSwerveModulePositions;

      return Rotation2d.fromRadians(gyroAngleSim);
    }

    Logger.recordOutput("Gyro Rotation2d", gyro.getRotation2d());
    return gyro.getRotation2d();
    // .plus(Rotation2d.fromRadians(Math.PI));
  }

  public synchronized void setRotation2d(Rotation2d rotation2d) {
    setOdometry(new Pose2d(getPose().getTranslation(), rotation2d));
  }
}

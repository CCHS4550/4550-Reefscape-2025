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
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.helpers.vision.PhotonVisionAprilTag;
import frc.helpers.vision.PhotonVisionSim;
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

  VisionIO vision;

  /** Module positions used for odometry */
  public SwerveModulePosition[] previousSwerveModulePositions = new SwerveModulePosition[4];

  public SwerveModulePosition[] currentSwerveModulePositions = new SwerveModulePosition[4];

  public SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

  public int sampleCountHF;
  public double[] sampleTimestampsHF;
  public SwerveModulePosition[][] swerveModulePositionsHF;

  // Initialize gyro
  public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private final Queue<Double> gyroContainer;

  public Rotation2d[] gyroAnglesHF = new Rotation2d[] {};
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

  public void robotStateInit() {
    if (Robot.isReal()) vision = PhotonVisionAprilTag.getInstance();
    if (Robot.isSimulation()) vision = PhotonVisionSim.getInstance();

    swerveModulePositions[0] =
        new SwerveModulePosition(
            0, new Rotation2d(SwerveDriveSubsystem.getInstance().frontRight.getTurnPosition()));
    swerveModulePositions[1] =
        new SwerveModulePosition(
            0, new Rotation2d(SwerveDriveSubsystem.getInstance().frontLeft.getTurnPosition()));
    swerveModulePositions[2] =
        new SwerveModulePosition(
            0, new Rotation2d(SwerveDriveSubsystem.getInstance().backRight.getTurnPosition()));
    swerveModulePositions[3] =
        new SwerveModulePosition(
            0, new Rotation2d(SwerveDriveSubsystem.getInstance().backLeft.getTurnPosition()));

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

  /** Update the pose estimator with Odometry and Gyro Data (HF Functional) */
  public synchronized void updateOdometryPose() {

    gameField.setRobotPose(getPose());
    lastPose = currentPose;

    /** Send the high frequency gyro to an array */
    gyroAnglesHF =
        gyroContainer.stream()
            .map((Double value) -> new Rotation2d(value))
            .toArray(Rotation2d[]::new);
    gyroContainer.clear();

    if (!gyro.isConnected()) {
      /**
       * If the gyro isn't connected, take the current values and estimate how much it has changed
       * since then.
       */
      gyroAnglesHF = gyroAnglesPlusSwerveModuleDeltasHF();
    }

    /** If high frequency odometry data is available, use it! */
    sampleCountHF =
        SwerveDriveSubsystem.getInstance().swerveModuleInputs[0].odometryTimestamps.length;
    sampleTimestampsHF =
        SwerveDriveSubsystem.getInstance().swerveModuleInputs[0].odometryTimestamps;

    if (sampleCountHF > 0 && gyro.isConnected()) {

      swerveModulePositionsHF = getSwerveModulePositionArrayHF();

      /** Update the SwerveDrivePoseEstimator with the Drivetrain encoders and such */
      for (int i = 0; i < sampleCountHF; i++) {

        // System.out.println("sampleTimeStamps size" + sampleTimestamps.length);
        // System.out.println("gyroAngle size" + gyroAngle.length);
        // System.out.println("swerveModulePositionsArray size" +
        // swerveModulePositionsArray.length);

        poseEstimator.updateWithTime(
            getSampleTimestampArrayClampedHF(i),
            getGyroAngleArrayClampedHF(i),
            getSwerveModulePositionsArrayClampedHF(i));
        Logger.recordOutput("High Frequency Gyro Angle", getGyroAngleArrayClampedHF(i));
        Logger.recordOutput(
            "High Frequency Odometry Positions", getSwerveModulePositionsArrayClampedHF(i));
      }

    } else {
      double currentTime = Timer.getFPGATimestamp();
      poseEstimator.updateWithTime(currentTime, getRotation2d(), swerveModulePositions);
    }

    currentPose = getPose();
    Logger.recordOutput("SwerveDrivePoseEstimator/Estimated Pose", getPose());
    Logger.recordOutput(
        "SwerveDrivePoseEstimator/Estimated Angle Degrees", getPose().getRotation().getDegrees());
    Logger.recordOutput(
        "SwerveDrivePoseEstimator/Estimated Angle Radians", getPose().getRotation().getRadians());
  }

  /** Update the pose estimator with PhotonVision Data */
  public synchronized void updateVisionPose() {
    /** Update the visionData to what the camera sees. */
    vision.updateInputs(visionInputs);

    for (int i = 0; i < visionInputs.poseEstimates.length; i++) {
      /** Add the Photonvision pose estimates */
      poseEstimator.addVisionMeasurement(
          visionInputs.poseEstimates[i], visionInputs.timestampArray[i]);
    }

    Logger.processInputs("Subsystem/Vision", visionInputs);
  }

  public synchronized void dashboardInit() {

    /* Put the Command Scheduler on SmartDashboard */
    SmartDashboard.putData(CommandScheduler.getInstance());

    /* Put all the subsystems on ShuffleBoard in their own "Subsystems" tab. */
    Shuffleboard.getTab("Subsystems").add("Swerve Drive", SwerveDriveSubsystem.getInstance());
    Shuffleboard.getTab("Subsystems").add("Algae", AlgaeSubsystem.getInstance());
    Shuffleboard.getTab("Subsystems").add("Arm", ArmSubsystem.getInstance());
    Shuffleboard.getTab("Subsystems").add("Elevator", ElevatorSubsystem.getInstance());
    Shuffleboard.getTab("Subsystems").add("Wrist", WristSubsystem.getInstance());
    Shuffleboard.getTab("Subsystems").add("Intake", IntakeSubsystem.getInstance());

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
            .add(
                SwerveDriveSubsystem.getInstance().frontRight.getName(),
                SwerveDriveSubsystem.getInstance().frontRight.getAbsoluteEncoderRadiansOffset())
            .getEntry();
    abs_Enc_FL_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(
                SwerveDriveSubsystem.getInstance().frontLeft.getName(),
                SwerveDriveSubsystem.getInstance().frontLeft.getAbsoluteEncoderRadiansOffset())
            .getEntry();
    abs_Enc_BR_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(
                SwerveDriveSubsystem.getInstance().backRight.getName(),
                SwerveDriveSubsystem.getInstance().backRight.getAbsoluteEncoderRadiansOffset())
            .getEntry();
    abs_Enc_BL_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(
                SwerveDriveSubsystem.getInstance().backLeft.getName(),
                SwerveDriveSubsystem.getInstance().backLeft.getAbsoluteEncoderRadiansOffset())
            .getEntry();

    enc_FR_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(
                SwerveDriveSubsystem.getInstance().frontRight.getName(),
                SwerveDriveSubsystem.getInstance().frontRight.getTurnPosition())
            .getEntry();
    enc_FL_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(
                SwerveDriveSubsystem.getInstance().frontLeft.getName(),
                SwerveDriveSubsystem.getInstance().frontLeft.getTurnPosition())
            .getEntry();
    enc_BR_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(
                SwerveDriveSubsystem.getInstance().backRight.getName(),
                SwerveDriveSubsystem.getInstance().backRight.getTurnPosition())
            .getEntry();
    enc_BL_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(
                SwerveDriveSubsystem.getInstance().backLeft.getName(),
                SwerveDriveSubsystem.getInstance().backLeft.getTurnPosition())
            .getEntry();

    abs_Enc_FR_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(
                SwerveDriveSubsystem.getInstance().frontRight.getName(),
                SwerveDriveSubsystem.getInstance().frontRight.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
    abs_Enc_FL_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(
                SwerveDriveSubsystem.getInstance().frontLeft.getName(),
                SwerveDriveSubsystem.getInstance().frontLeft.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
    abs_Enc_BR_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(
                SwerveDriveSubsystem.getInstance().backRight.getName(),
                SwerveDriveSubsystem.getInstance().backRight.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
    abs_Enc_BL_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(
                SwerveDriveSubsystem.getInstance().backLeft.getName(),
                SwerveDriveSubsystem.getInstance().backLeft.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
  }

  public synchronized void updateSwerveModuleEncoders() {
    abs_Enc_FR_Offset_Entry.setDouble(
        SwerveDriveSubsystem.getInstance().frontRight.getAbsoluteEncoderRadiansOffset());
    abs_Enc_FL_Offset_Entry.setDouble(
        SwerveDriveSubsystem.getInstance().frontLeft.getAbsoluteEncoderRadiansOffset());
    abs_Enc_BR_Offset_Entry.setDouble(
        SwerveDriveSubsystem.getInstance().backRight.getAbsoluteEncoderRadiansOffset());
    abs_Enc_BL_Offset_Entry.setDouble(
        SwerveDriveSubsystem.getInstance().backLeft.getAbsoluteEncoderRadiansOffset());

    abs_Enc_FR_Raw_Entry.setDouble(
        SwerveDriveSubsystem.getInstance().frontRight.getAbsoluteEncoderRadiansNoOffset());
    abs_Enc_FL_Raw_Entry.setDouble(
        SwerveDriveSubsystem.getInstance().frontLeft.getAbsoluteEncoderRadiansNoOffset());
    abs_Enc_BR_Raw_Entry.setDouble(
        SwerveDriveSubsystem.getInstance().backRight.getAbsoluteEncoderRadiansNoOffset());
    abs_Enc_BL_Raw_Entry.setDouble(
        SwerveDriveSubsystem.getInstance().backLeft.getAbsoluteEncoderRadiansNoOffset());

    enc_FR_pos_Entry.setDouble(SwerveDriveSubsystem.getInstance().frontRight.getTurnPosition());
    enc_FL_pos_Entry.setDouble(SwerveDriveSubsystem.getInstance().frontLeft.getTurnPosition());
    enc_BR_pos_Entry.setDouble(SwerveDriveSubsystem.getInstance().backRight.getTurnPosition());
    enc_BL_pos_Entry.setDouble(SwerveDriveSubsystem.getInstance().backLeft.getTurnPosition());
  }

  /** Update odometry, not HF */
  public SwerveModulePosition[] updateSwerveModulePositionsPeriodic() {
    swerveModulePositions[0] =
        new SwerveModulePosition(
            SwerveDriveSubsystem.getInstance().frontRight.getDrivePosition(),
            new Rotation2d(SwerveDriveSubsystem.getInstance().frontRight.getTurnPosition()));
    swerveModulePositions[1] =
        new SwerveModulePosition(
            SwerveDriveSubsystem.getInstance().frontLeft.getDrivePosition(),
            new Rotation2d(SwerveDriveSubsystem.getInstance().frontLeft.getTurnPosition()));
    swerveModulePositions[2] =
        new SwerveModulePosition(
            SwerveDriveSubsystem.getInstance().backRight.getDrivePosition(),
            new Rotation2d(SwerveDriveSubsystem.getInstance().backRight.getTurnPosition()));
    swerveModulePositions[3] =
        new SwerveModulePosition(
            SwerveDriveSubsystem.getInstance().backLeft.getDrivePosition(),
            new Rotation2d(SwerveDriveSubsystem.getInstance().backLeft.getTurnPosition()));

    Logger.recordOutput("currentModulePositions", swerveModulePositions);

    return swerveModulePositions;
  }

  /** Read HF odometry data */
  public synchronized SwerveModulePosition[][] getSwerveModulePositionArrayHF() {

    int[] numbers = {
      SwerveDriveSubsystem.getInstance().swerveModuleInputs[0].odometryDrivePositionsMeters.length,
      SwerveDriveSubsystem.getInstance().swerveModuleInputs[1].odometryDrivePositionsMeters.length,
      SwerveDriveSubsystem.getInstance().swerveModuleInputs[2].odometryDrivePositionsMeters.length,
      SwerveDriveSubsystem.getInstance().swerveModuleInputs[3].odometryDrivePositionsMeters.length,
      SwerveDriveSubsystem.getInstance().swerveModuleInputs[0].odometryTurnPositions.length,
      SwerveDriveSubsystem.getInstance().swerveModuleInputs[1].odometryTurnPositions.length,
      SwerveDriveSubsystem.getInstance().swerveModuleInputs[2].odometryTurnPositions.length,
      SwerveDriveSubsystem.getInstance().swerveModuleInputs[3].odometryTurnPositions.length,
    };

    int min = Arrays.stream(numbers).min().getAsInt();

    SwerveModulePosition[][] positions = new SwerveModulePosition[min][4];

    for (int i = 0; i < min; i++) {
      positions[i][0] =
          new SwerveModulePosition(
              SwerveDriveSubsystem.getInstance()
                  .swerveModuleInputs[0]
                  .odometryDrivePositionsMeters[i],
              SwerveDriveSubsystem.getInstance().swerveModuleInputs[0].odometryTurnPositions[i]);
      positions[i][1] =
          new SwerveModulePosition(
              SwerveDriveSubsystem.getInstance()
                  .swerveModuleInputs[1]
                  .odometryDrivePositionsMeters[i],
              SwerveDriveSubsystem.getInstance().swerveModuleInputs[1].odometryTurnPositions[i]);
      positions[i][2] =
          new SwerveModulePosition(
              SwerveDriveSubsystem.getInstance()
                  .swerveModuleInputs[2]
                  .odometryDrivePositionsMeters[i],
              SwerveDriveSubsystem.getInstance().swerveModuleInputs[2].odometryTurnPositions[i]);
      positions[i][3] =
          new SwerveModulePosition(
              SwerveDriveSubsystem.getInstance()
                  .swerveModuleInputs[3]
                  .odometryDrivePositionsMeters[i],
              SwerveDriveSubsystem.getInstance().swerveModuleInputs[3].odometryTurnPositions[i]);
    }

    if (min != 0) swerveModulePositions = positions[min - 1];

    return positions;
  }

  /**
   * Estimate gyro angle based on last known gyro inputs and change in module positions. Will get
   * less accurate over time.
   */
  public synchronized Rotation2d[] gyroAnglesPlusSwerveModuleDeltasHF() {

    List<Rotation2d> gyroAnglesPlusDeltasHF = new ArrayList<>();

    SwerveModulePosition[] previousPositions = swerveModulePositions;

    for (int i = 0; i < (gyroAnglesHF.length); i++) {
      SwerveModulePosition[] samplePositions = swerveModulePositionsHF[i];

      Twist2d deltas =
          Constants.SwerveConstants.DRIVE_KINEMATICS.toTwist2d(previousPositions, samplePositions);

      for (int j = 0; j < gyroAnglesHF.length; j++)
        gyroAnglesHF[j] = gyroAnglesHF[gyroAnglesHF.length - 1];

      gyroAnglesPlusDeltasHF.add(gyroAnglesHF[i].plus(Rotation2d.fromRadians(deltas.dtheta)));

      previousPositions = samplePositions;
    }

    return gyroAnglesPlusDeltasHF.toArray(new Rotation2d[0]);
  }

  /**
   * Clamping methods for allowing data reading past array length, since lengths are inconsistent.
   */
  public SwerveModulePosition[] getSwerveModulePositionsArrayClampedHF(int index) {
    if (index > swerveModulePositionsHF.length - 1) {
      return swerveModulePositionsHF[swerveModulePositionsHF.length - 1];
    }
    return swerveModulePositionsHF[index];
  }

  public double getSampleTimestampArrayClampedHF(int index) {
    if (index > sampleTimestampsHF.length - 1) {

      return sampleTimestampsHF[sampleTimestampsHF.length - 1];
    }
    return sampleTimestampsHF[index];
  }

  public Rotation2d getGyroAngleArrayClampedHF(int index) {
    if (index > gyroAnglesHF.length - 1) {
      return gyroAnglesHF[gyroAnglesHF.length - 1];
    }
    return gyroAnglesHF[index];
  }

  /**
   * Gets the position of the robot in Pose2d format. Uses odometer reading. Includes the x, y, and
   * theta values of the robot.
   *
   * @return The Pose2d of the robot.
   */
  public synchronized Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public synchronized void printPos2d() {
    System.out.println(poseEstimator.getEstimatedPosition());
  }

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
  public synchronized void setOdometry(Translation2d translation) {
    poseEstimator.resetPosition(
        getRotation2d(), swerveModulePositions, new Pose2d(translation, getRotation2d()));
  }

  public synchronized void setOdometry(Pose2d pose) {
    poseEstimator.resetPosition(pose.getRotation(), swerveModulePositions, pose);
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

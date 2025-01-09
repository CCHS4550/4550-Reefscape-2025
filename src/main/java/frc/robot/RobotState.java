// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import frc.helpers.Vision.VisionData;
// import frc.maps.Constants;
import frc.robot.maps.Constants;
import frc.robot.subsystems.DriveTrain.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonUtils;

/** RobotState is used to retrieve information about the robot's state in other classes. */
public class RobotState {

  public static RobotState instance;
  // Initialize gyro
  public AHRS gyro = new AHRS(SPI.Port.kMXP);

  double angleSim = 0;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  public final Field2d gameField = new Field2d();

  /** Technically pose ESTIMATES */
  public Pose2d lastPose = new Pose2d();

  public Pose2d currentPose = new Pose2d();
  // public Pose2d simPose = new Pose2d();

  // SwerveDriveOdometry odometer;
  public SwerveDrivePoseEstimator poseEstimator;
//   public PhotonPoseEstimator photonPoseEstimator;

  public final VisionData visionData = new VisionData();

  public void poseInit() {

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(gyro.getAngle()).unaryMinus(),
            swerve.swerveModulePositionsReal,
            new Pose2d(0, 0, new Rotation2d(0)));

    if (RobotBase.isSimulation()) {
      poseEstimator =
          new SwerveDrivePoseEstimator(
              Constants.SwerveConstants.DRIVE_KINEMATICS,
              getRotation2d(),
              swerve.swerveModulePositionsSim,
              new Pose2d(0, 0, new Rotation2d(0)));
    }
  }

  public synchronized void updateOdometryPose() {

    gameField.setRobotPose(getPose());
    // Logger.recordOutput("REAL Pose", getPose());

    lastPose = currentPose;

    /** Update the SwerveDrivePoseEstimator with the Drivetrain encoders and such */
    if (RobotBase.isReal()) {
      poseEstimator.updateWithTime(
          Timer.getFPGATimestamp(),
          getRotation2d(),
          swerve.swerveModulePositionsReal);
    } else if (RobotBase.isSimulation()) {

      poseEstimator.updateWithTime(
          Timer.getFPGATimestamp(),
          getRotation2d(),
          swerve.swerveModulePositionsSim);
    }

    currentPose = getPose();
    Logger.recordOutput("Estimated Pose", getPose());
    Logger.recordOutput("Estimated Angle", getPose().getRotation().getDegrees());
  }

//   public void updateVisionPose() {
//     /** Update the visionData to what the camera sees. */
//     if (Robot.isReal()) {
//       PhotonVision.getInstance().updateData(visionData, getPose());

//       for (int i = 0; i < visionData.poseEstimates.size(); i++) {
//         /** Add the Photonvision pose estimates */
//         poseEstimator.addVisionMeasurement(visionData.poseEstimates.get(i), visionData.timestamp);
//       }
//     } else if (Robot.isSimulation()) {
//       PhotonVision.getInstance().visionSim.update(getPose());
//     }
//   }

  public synchronized void dashboardInit() {

    /* Put the Command Scheduler on SmartDashboard */
    SmartDashboard.putData(CommandScheduler.getInstance());

    /* Put all the subsystems on ShuffleBoard in their own "Subsystems" tab. */
    Shuffleboard.getTab("Subsystems").add("Swerve Drive", swerve);
    // Shuffleboard.getTab("Subsystems").add("Indexing Subsystem", IndexingSubsystem.getInstance());
    // Shuffleboard.getTab("Subsystems")
    //     .add("Declination Subsystem", DeclinationSubsystem.getInstance());
    // Shuffleboard.getTab("Subsystems").add("Pneumatics System", PneumaticsSystem.getInstance());
    // Shuffleboard.getTab("Subsystems")
    //     .add("Right Ascension Subsystem", RightAscensionSubsystem.getInstance());
    // Shuffleboard.getTab("Subsystems").add("Aim Simulator", AimSimulator.getInstance());
    // Shuffleboard.getTab("Subsystems").add("Lights", Lights.getInstance());

    /* Put the Pose Estimators on Dashboards */
    SmartDashboard.putData("Field", gameField);
    // Shuffleboard.getTab("Tab 6").add(m_field_poseestimator);
  }

  public synchronized void updateDashboard() {

    SmartDashboard.putNumber("X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber(
        "Rads", poseEstimator.getEstimatedPosition().getRotation().getRadians());
    SmartDashboard.putNumber(
        "Angle", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
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
  // private GenericEntry enc_FR_vel_Entry, enc_FL_vel_Entry, enc_BR_vel_Entry,
  // enc_BL_vel_Entry;

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

  public synchronized void moduleEncodersInit(SwerveDriveSubsystem swerveDrive) {

    // IndexingSubsystem indexer,
    // DeclinationSubsystem declination,
    // PneumaticsSystem pneumatics,

    abs_Enc_FR_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(
                swerveDrive.frontRight.getName(),
                swerveDrive.frontRight.getAbsoluteEncoderRadiansOffset())
            .getEntry();
    abs_Enc_FL_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(
                swerveDrive.frontLeft.getName(),
                swerveDrive.frontLeft.getAbsoluteEncoderRadiansOffset())
            .getEntry();
    abs_Enc_BR_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(
                swerveDrive.backRight.getName(),
                swerveDrive.backRight.getAbsoluteEncoderRadiansOffset())
            .getEntry();
    abs_Enc_BL_Offset_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_offset_list.getTitle())
            .add(
                swerveDrive.backLeft.getName(),
                swerveDrive.backLeft.getAbsoluteEncoderRadiansOffset())
            .getEntry();

    enc_FR_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(swerveDrive.frontRight.getName(), swerveDrive.frontRight.getTurnPosition())
            .getEntry();
    enc_FL_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(swerveDrive.frontLeft.getName(), swerveDrive.frontLeft.getTurnPosition())
            .getEntry();
    enc_BR_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(swerveDrive.backRight.getName(), swerveDrive.backRight.getTurnPosition())
            .getEntry();
    enc_BL_pos_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(turn_encoders_positions.getTitle())
            .add(swerveDrive.backLeft.getName(), swerveDrive.backLeft.getTurnPosition())
            .getEntry();

    abs_Enc_FR_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(
                swerveDrive.frontRight.getName(),
                swerveDrive.frontRight.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
    abs_Enc_FL_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(
                swerveDrive.frontLeft.getName(),
                swerveDrive.frontLeft.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
    abs_Enc_BR_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(
                swerveDrive.backRight.getName(),
                swerveDrive.backRight.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
    abs_Enc_BL_Raw_Entry =
        Shuffleboard.getTab("Encoders")
            .getLayout(absolute_encoders_no_offset_list.getTitle())
            .add(
                swerveDrive.backLeft.getName(),
                swerveDrive.backLeft.getAbsoluteEncoderRadiansNoOffset())
            .getEntry();
  }

  public synchronized void aimerShuffleBoardInit(
    //   IndexingSubsystem indexer,
    //   DeclinationSubsystem declination,
    //   PneumaticsSystem pneumatics,
    //   RightAscensionSubsystem rightAscension,
    //   AimSimulator aimer
      ) {
    // yActual =
    //     Shuffleboard.getTab("Aimer")
    //         .add("Y Actual", declination.declination1.getPosition())
    //         .getEntry();

    // yGoal = Shuffleboard.getTab("Aimer").add("Y Goal", declination.getGoal().position).getEntry();

    // xActual =
    //     Shuffleboard.getTab("Aimer")
    //         .add("X Actual", rightAscension.rightAscensionMotor.getPosition())
    //         .getEntry();

    // xGoal =
    //     Shuffleboard.getTab("Aimer").add("X Goal", rightAscension.getGoal().position).getEntry();

    // barrelActual =
    //     Shuffleboard.getTab("Aimer")
    //         .add("Actual Barrel Angle", indexer.indexMotor.getPosition())
    //         .getEntry();

    // barrelGoal =
    //     Shuffleboard.getTab("Aimer").add("Goal Barrel Angle", aimer.barrelAngle).getEntry();
  }

  public synchronized void updateModuleEncoders(SwerveDriveSubsystem swerve) {
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

  private double[] moduleDistanceSim = {0, 0, 0, 0};

  // private SwerveModulePosition[] initialModulePositions = {
  //   new SwerveModulePosition(),
  //   new SwerveModulePosition(),
  //   new SwerveModulePosition(),
  //   new SwerveModulePosition(),
  // };
  // private SwerveModulePosition[] deltaModulePositions = {
  //   new SwerveModulePosition(),
  //   new SwerveModulePosition(),
  //   new SwerveModulePosition(),
  //   new SwerveModulePosition(),
  // };

  private double previousTime = Timer.getFPGATimestamp();

  public synchronized void updateModulePositions(SwerveDriveSubsystem swerve) {

            swerve.swerveModulePositionsReal[0] =
        new SwerveModulePosition(
            swerve.frontRight.getDrivePosition(),
            new Rotation2d(swerve.frontRight.getTurnPosition()));
            swerve.swerveModulePositionsReal[1] =
        new SwerveModulePosition(
            swerve.frontLeft.getDrivePosition(),
            new Rotation2d(swerve.frontLeft.getTurnPosition()));
            swerve.swerveModulePositionsReal[2] =
        new SwerveModulePosition(
            swerve.backRight.getDrivePosition(),
            new Rotation2d(swerve.backRight.getTurnPosition()));
            swerve.swerveModulePositionsReal[3] =
        new SwerveModulePosition(
            swerve.backLeft.getDrivePosition(),
            new Rotation2d(swerve.backLeft.getTurnPosition()));

    if (RobotBase.isSimulation()) {

      // initialModulePositions = SwerveDrive.getInstance().swerveModulePositionsSim;
      double currentTime = Timer.getFPGATimestamp();

      double dt = currentTime - previousTime;
      Logger.recordOutput("dt", dt);

      moduleDistanceSim[0] +=
      swerve.desiredModuleStates[0].speedMetersPerSecond * dt;
      moduleDistanceSim[1] +=
      swerve.desiredModuleStates[1].speedMetersPerSecond * dt;
      moduleDistanceSim[2] +=
      swerve.desiredModuleStates[2].speedMetersPerSecond * dt;
      moduleDistanceSim[3] +=
      swerve.desiredModuleStates[3].speedMetersPerSecond * dt;

      swerve.swerveModulePositionsSim[0] =
          new SwerveModulePosition(
              moduleDistanceSim[0], swerve.desiredModuleStates[0].angle);
              swerve.swerveModulePositionsSim[1] =
          new SwerveModulePosition(
              moduleDistanceSim[1], swerve.desiredModuleStates[1].angle);
              swerve.swerveModulePositionsSim[2] =
          new SwerveModulePosition(
              moduleDistanceSim[2], swerve.desiredModuleStates[2].angle);
              swerve.swerveModulePositionsSim[3] =
          new SwerveModulePosition(
              moduleDistanceSim[3], swerve.desiredModuleStates[3].angle);

      // deltaModulePositions[0] =
      //     new SwerveModulePosition(
      //         SwerveDrive.getInstance().swerveModulePositionsSim[0].distanceMeters
      //             - initialModulePositions[0].distanceMeters,
      //         SwerveDrive.getInstance()
      //             .swerveModulePositionsSim[0]
      //             .angle
      //             .minus(initialModulePositions[0].angle));
      // deltaModulePositions[1] =
      //     new SwerveModulePosition(
      //         SwerveDrive.getInstance().swerveModulePositionsSim[1].distanceMeters
      //             - initialModulePositions[1].distanceMeters,
      //         SwerveDrive.getInstance()
      //             .swerveModulePositionsSim[1]
      //             .angle
      //             .minus(initialModulePositions[1].angle));
      // deltaModulePositions[2] =
      //     new SwerveModulePosition(
      //         SwerveDrive.getInstance().swerveModulePositionsSim[2].distanceMeters
      //             - initialModulePositions[2].distanceMeters,
      //         SwerveDrive.getInstance()
      //             .swerveModulePositionsSim[2]
      //             .angle
      //             .minus(initialModulePositions[2].angle));
      // deltaModulePositions[3] =
      //     new SwerveModulePosition(
      //         SwerveDrive.getInstance().swerveModulePositionsSim[3].distanceMeters
      //             - initialModulePositions[3].distanceMeters,
      //         SwerveDrive.getInstance()
      //             .swerveModulePositionsSim[3]
      //             .angle
      //             .minus(initialModulePositions[3].angle));

      // Logger.recordOutput("initialModulePositions", initialModulePositions);
      // Logger.recordOutput(
      //     "finalModulePositions", SwerveDrive.getInstance().swerveModulePositionsSim);

      previousTime = currentTime;
    }
  }

  /** Pose Helper Methods */

  /**
   * Gets the position of the robot in Pose2d format. Uses odometer reading. Includes the x, y, and
   * theta values of the robot.
   *
   * @return The Pose2d of the robot.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // /** If Using REFERENCE POSE STRAT */
  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
  //   photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
  //   return photonPoseEstimator.update();
  // }

  public void printPos2d() {
    System.out.println(poseEstimator.getEstimatedPosition());
  }

  // Returns the estimated transformation over the next tick (The change in
  // position)
  private Transform2d getTickFutureTransform() {
    return new Transform2d(
        new Translation2d(
            swerve.getFieldVelocity().vxMetersPerSecond * 0.02,
            swerve.getFieldVelocity().vyMetersPerSecond * 0.02),
        new Rotation2d(0.0));
  }

  // Returns the estimated robot position a tick from the current time (Theoretically?)
  private Pose2d getFutureTickPose() {
    return getPose().plus(getTickFutureTransform().inverse());
  }

//   /** Can use this for turret. */
//   public Rotation2d getAngleBetweenCurrentAndTargetPose(Pose2d targetPose) {
//     Rotation2d targetYaw = PhotonUtils.getYawToPose(getPose(), targetPose);
//     return targetYaw;
//   }

  /** Odometry */

  /**
   * Resets the odometer readings using the gyro, SwerveModulePositions (defined in constructor),
   * and Pose2d. Also used in AutonomousScheme.java
   */
  public void setOdometry() {
    poseEstimator.resetPosition(
        getRotation2d(), swerve.swerveModulePositionsReal, getPose());
    if (RobotBase.isSimulation()) {
      poseEstimator.resetPosition(
          getRotation2d(), swerve.swerveModulePositionsSim, getPose());
    }
  }

  /**
   * Resets the odometer readings using the gyro, SwerveModulePositions (defined in constructor),
   * and Pose2d. Also used in AutonomousScheme.java
   *
   * @param pos the Pose2d to set the odometry
   */
  public void setOdometry(Pose2d pos) {
    poseEstimator.resetPosition(
        getRotation2d(),
        // pos.getRotation(),
        swerve.swerveModulePositionsReal,
        pos);

    if (RobotBase.isSimulation()) {
      poseEstimator.resetPosition(
          getRotation2d(),
          // pos.getRotation(),
          swerve.swerveModulePositionsSim,
          pos);
    }
  }
  /** Gyroscope Methods (NavX) */
  public void zeroHeading() {
    gyro.reset();
    angleSim = 0;
    setOdometry(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(0)));
  }

  public double getPitch() {
    return gyro.getPitch() - 1.14;
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  /**
   * Method to get the facing direction of the gyro.
   *
   * @return The facing direction of the gyro, between -360 and 360 degrees.
   */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public Rotation2d getPoseRotation2d() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  public double getPoseAngleDegrees() {
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  public double getPoseAngleRadians() {
    return poseEstimator.getEstimatedPosition().getRotation().getRadians();
  }

  /**
   * Gets the Rotation2d value of the facing direction of the robot.
   *
   * @return The facing direction of the robot in Rotation2d format.
   */
  SwerveDriveWheelPositions previousPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });

  SwerveDriveWheelPositions currentPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });

  /**
   * Specifically for the poseEstimator. Anywhere else, get the poseEstimator's estimated heading,
   * or it'll mess things up.
   *
   * @return
   */
  public Rotation2d getRotation2d() {

    if (RobotBase.isSimulation()) {

      currentPositions =
          new SwerveDriveWheelPositions(SwerveDrive.getInstance().swerveModulePositionsSim);

      angleSim +=
          Constants.SwerveConstants.DRIVE_KINEMATICS.toTwist2d(previousPositions, currentPositions)
              .dtheta;

      Logger.recordOutput("Sim angle", angleSim);
      Logger.recordOutput("Sim Rotation2d", Rotation2d.fromRadians(angleSim));

      previousPositions = currentPositions;

      return Rotation2d.fromRadians(angleSim);
    }

    // System.out.println(simAngleReal);

    // simulatedAngleRadians %= (Math.PI * 2);

    Logger.recordOutput("Gyro Rotation2d", gyro.getRotation2d());
    return gyro.getRotation2d();
    // .plus(Rotation2d.fromRadians(Math.PI));
  }

  public void setRotation2d(Rotation2d rotation2d) {
    setOdometry(new Pose2d(getPose().getTranslation(), rotation2d));
  }

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
  }

  public void periodic() {}
}
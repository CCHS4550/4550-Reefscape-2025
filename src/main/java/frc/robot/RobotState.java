// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.CustomAutoChooser;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.algae.AlgaeIOInputsAutoLogged;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.climber.ClimberIOInputsAutoLogged;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.superstructure.arm.ArmIOInputsAutoLogged;
import frc.robot.subsystems.superstructure.arm.ArmSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.wrist.WristIOInputsAutoLogged;
import frc.robot.subsystems.superstructure.wrist.WristSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleInputsAutoLogged;
import frc.robot.vision.VisionIO;
import frc.robot.vision.VisionIOInputsAutoLogged;
import frc.util.HighFrequencyThread;
import frc.util.maps.Constants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Queue;
import java.util.function.BooleanSupplier;
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

  private RobotState() {
    poseInitialized = false;
  }

  private AlgaeSubsystem algae;
  private ArmSubsystem arm;
  private ClimberSubsystem climber;
  private ElevatorSubsystem elevator;
  private IntakeSubsystem intake;
  private SwerveDriveSubsystem swerve;
  private WristSubsystem wrist;

  private VisionIO vision;

  private Superstructure superstructure;

  public SuperState currentSuperState;

  /** NavX Gyroscope */
  // private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  /** Pigeon 2 Gyroscope (Better) */
  public final Pigeon2 pigeonGyro = new Pigeon2(Constants.MotorConstants.PIGEON);

  public AlgaeIOInputsAutoLogged algaeInputs;
  public ArmIOInputsAutoLogged armInputs;
  public ClimberIOInputsAutoLogged climberInputs;
  public ElevatorIOInputsAutoLogged elevatorInputs;
  public IntakeIOInputsAutoLogged intakeInputs;
  public SwerveModuleInputsAutoLogged[] moduleInputs;
  public WristIOInputsAutoLogged wristInputs;

  public final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

  /** Module positions used for odometry (not HF) */
  public SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

  public BooleanSupplier allowSubsystemMovement = () -> true;
  public BooleanSupplier moveElevator = () -> true;
  public BooleanSupplier moveArm = () -> true;
  public BooleanSupplier moveWrist = () -> true;

  public boolean useHF = true;

  public boolean poseInitialized;

  /** High frequency odometry objects (HF) */
  public static int sampleCountHF;

  public double[] sampleTimestampsHF;
  public SwerveModulePosition[][] swerveModulePositionsHF;

  private StatusSignal<Angle> yaw;
  private Queue<Double> gyroContainer;
  private Queue<Double> gyroTimestampContainer;
  public Rotation2d[] gyroAnglesHF = new Rotation2d[] {};
  public double[] gyroTimestampsHF = new double[] {};

  /** Pose estimation objects */
  public final Field2d gameField = new Field2d();

  public SwerveDrivePoseEstimator poseEstimator;

  /** Methods for simulation */
  public double gyroAngleSim = 0;

  public SwerveModulePosition[] previousSwerveModulePositions = new SwerveModulePosition[4];
  public SwerveModulePosition[] currentSwerveModulePositions = new SwerveModulePosition[4];

  public void robotStateInit(
      AlgaeSubsystem algae,
      ArmSubsystem arm,
      ClimberSubsystem climber,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake,
      SwerveDriveSubsystem swerve,
      WristSubsystem wrist,
      VisionIO vision,
      Superstructure superstructure) {

    this.algae = algae;
    this.arm = arm;
    this.climber = climber;
    this.elevator = elevator;
    this.intake = intake;
    this.swerve = swerve;
    this.wrist = wrist;

    this.vision = vision;

    this.superstructure = superstructure;

    this.currentSuperState = superstructure.currentSuperState;

    algaeInputs = algae.algaeInputs;
    armInputs = arm.armInputs;
    climberInputs = climber.climberInputs;
    elevatorInputs = elevator.elevatorInputs;
    intakeInputs = intake.intakeInputs;
    moduleInputs = swerve.swerveModuleInputs;
    wristInputs = wrist.wristInputs;

    yaw = pigeonGyro.getYaw();
    pigeonGyro.getConfigurator().apply(new Pigeon2Configuration());
    pigeonGyro.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Constants.SwerveConstants.ODOMETRY_FREQUENCY);
    pigeonGyro.optimizeBusUtilization();
    gyroTimestampContainer = HighFrequencyThread.getInstance().makeTimestampContainer();
    gyroContainer =
        HighFrequencyThread.getInstance().registerInput(() -> getRotation2d().getDegrees());

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

  public CustomAutoChooser autoChooserInit() {
    return new CustomAutoChooser(
        algae, arm, climber, elevator, intake, swerve, wrist, vision, superstructure);
  }

  public void poseInit() {

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.DRIVE_KINEMATICS,
            getRotation2d(),
            swerveModulePositions,
            new Pose2d(0, 0, new Rotation2d()));

    poseInitialized = true;
  }

  /** Update the pose estimator with Odometry and Gyro Data (HF Functional) */
  public void updateOdometryPose() {

    gameField.setRobotPose(getPose());

    if (useHF) {
      /** Send the high frequency gyro to an array */
      gyroAnglesHF =
          gyroContainer.stream()
              .map((Double value) -> Rotation2d.fromDegrees(value))
              .toArray(Rotation2d[]::new);

      gyroTimestampsHF =
          gyroTimestampContainer.stream().mapToDouble((Double value) -> value).toArray();

      gyroContainer.clear();
      gyroTimestampContainer.clear();

      /** If gyro disconnected, estimate its value. */
      if (!pigeonGyro.isConnected()) {
        gyroAnglesHF = gyroAnglesPlusSwerveModuleDeltasHF();
      }

      /** Use any one of the module timestamps as reference for everything! */
      sampleTimestampsHF = getSampleTimestampArrayHF();
      swerveModulePositionsHF = getSwerveModulePositionArrayHF();

      /**
       * If, and only if, their size is greater than zero, do you continue, since it'll crash the
       * code otherwise. Sometimes the HF doesn't keep up and there isn't data.
       */
      if (swerveModulePositionsHF.length > 0
          && sampleTimestampsHF.length > 0
          && gyroAnglesHF.length > 0
          && pigeonGyro.isConnected()) {

        /** Update the SwerveDrivePoseEstimator with the Drivetrain encoders and such */
        for (int i = 0; i < getMinLengthHF(); i++) {

          poseEstimator.updateWithTime(
              clampSampleTimestampsHF(sampleTimestampsHF, i),
              clampGyroAnglesHF(gyroAnglesHF, i),
              clampSwerveModulePositionsHF(swerveModulePositionsHF, i));
          Logger.recordOutput("HF/High Frequency Gyro Angle", clampGyroAnglesHF(gyroAnglesHF, i));
          Logger.recordOutput(
              "HF/High Frequency Odometry Positions",
              clampSwerveModulePositionsHF(swerveModulePositionsHF, i));

          // System.out.println(pigeonGyro.getRotation2d().getDegrees());
          // System.out.println(yaw.getValueAsDouble());
        }
      }

    } else {

      poseEstimator.updateWithTime(Timer.getTimestamp(), getRotation2d(), swerveModulePositions);
    }

    Logger.recordOutput("SwerveDrivePoseEstimator/Estimated Pose", getPose());
    Logger.recordOutput(
        "SwerveDrivePoseEstimator/Estimated Angle Degrees", getPose().getRotation().getDegrees());
    Logger.recordOutput(
        "SwerveDrivePoseEstimator/Estimated Angle Radians", getPose().getRotation().getRadians());
  }

  /** Update the pose estimator with PhotonVision Data */
  public void updateVisionPose() {

    /** Update the visionData to what the camera sees. */
    vision.updateInputs(visionInputs);

    for (int i = 0; i < visionInputs.poseEstimates.length; i++) {
      /** Add the Photonvision pose estimates */
      poseEstimator.addVisionMeasurement(
          visionInputs.poseEstimates[i], visionInputs.timestampArray[i]);
    }

    Logger.processInputs("Subsystem/Vision", visionInputs);
  }

  public void dashboardInit() {

    /* Put the Command Scheduler on SmartDashboard */
    SmartDashboard.putData(CommandScheduler.getInstance());

    /* Put all the subsystems on ShuffleBoard in their own "Subsystems" tab. */

    Shuffleboard.getTab("Subsystem").add("Algae", algae);
    Shuffleboard.getTab("Subsystem").add("Arm", arm);
    Shuffleboard.getTab("Subsystem").add("Climber", climber);
    Shuffleboard.getTab("Subsystem").add("Elevator", elevator);
    Shuffleboard.getTab("Subsystem").add("Intake", intake);
    Shuffleboard.getTab("Subsystem").add("Swerve Drive", swerve);
    Shuffleboard.getTab("Subsystem").add("Wrist", wrist);

    SmartDashboard.putNumber("Robot X Position", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Robot Y Position", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber(
        "Robot Rads Angle", poseEstimator.getEstimatedPosition().getRotation().getRadians());
    SmartDashboard.putNumber(
        "Robot Degrees Angle", poseEstimator.getEstimatedPosition().getRotation().getDegrees());

    /* Put the Pose Estimators on Dashboards */
    SmartDashboard.putData("Field", gameField);
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

  public void swerveModuleEncodersInit() {

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

  public void swerveModuleEncodersPeriodic() {
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

  /** Update odometry, not HF */
  public SwerveModulePosition[] updateSwerveModulePositionsPeriodic() {
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

    return swerveModulePositions;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerve.getRobotRelativeSpeeds();
  }

  /** This is very important to make sure all the data is the same in quantity. */
  public int getMinLengthHF() {

    int[] lengths = {
      swerve.swerveModuleInputs[0].odometryDrivePositionsMeters.length,
      swerve.swerveModuleInputs[1].odometryDrivePositionsMeters.length,
      swerve.swerveModuleInputs[2].odometryDrivePositionsMeters.length,
      swerve.swerveModuleInputs[3].odometryDrivePositionsMeters.length,
      swerve.swerveModuleInputs[0].odometryTurnPositions.length,
      swerve.swerveModuleInputs[1].odometryTurnPositions.length,
      swerve.swerveModuleInputs[2].odometryTurnPositions.length,
      swerve.swerveModuleInputs[3].odometryTurnPositions.length,
    };

    return Arrays.stream(lengths).min().getAsInt();
  }

  /** Read HF odometry data */
  public SwerveModulePosition[][] getSwerveModulePositionArrayHF() {

    int min = getMinLengthHF();

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

    if (min > 0) swerveModulePositions = positions[min - 1];

    return positions;
  }

  public double[] getSampleTimestampArrayHF() {
    int min = getMinLengthHF();
    // List<Integer> allTimestamps = new ArrayList<>();
    double[][] allTimestamps = {
      swerve.swerveModuleInputs[0].odometryTimestamps,
      swerve.swerveModuleInputs[1].odometryTimestamps,
      swerve.swerveModuleInputs[2].odometryTimestamps,
      swerve.swerveModuleInputs[3].odometryTimestamps,
    };

    for (double[] timestamps : allTimestamps) {
      if (timestamps.length >= min) return timestamps;
    }

    return new double[0];
  }

  /**
   * Estimate gyro angle based on last known gyro inputs and change in module positions. Will get
   * less accurate over time. // MOSTLY UNUSED //
   */
  public Rotation2d[] gyroAnglesPlusSwerveModuleDeltasHF() {

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
  public static SwerveModulePosition[] clampSwerveModulePositionsHF(
      SwerveModulePosition[][] positions, int index) {
    if (index > positions.length - 1 && positions.length > 0) {
      return positions[positions.length - 1];
    } else return positions[index];
  }

  public static double clampSampleTimestampsHF(double[] timestamps, int index) {
    if (index > timestamps.length - 1) {

      return timestamps[timestamps.length - 1];
    }
    return timestamps[index];
  }

  public static Rotation2d clampGyroAnglesHF(Rotation2d[] angles, int index) {
    if (index > angles.length - 1) {
      return angles[angles.length - 1];
    }
    return angles[index];
  }

  public void resetPIDControllers() {
    arm.resetPID();
    elevator.resetPID();
    wrist.resetPID();
  }

  /**
   * Gets the position of the robot in Pose2d format. Uses odometer reading. Includes the x, y, and
   * theta values of the robot.
   *
   * @return The Pose2d of the robot.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void printPos2d() {
    System.out.println(poseEstimator.getEstimatedPosition());
  }

  /**
   * Resets the odometer readings using the gyro, SwerveModulePositions (defined in constructor),
   * and Pose2d. Also used in AutonomousScheme.java
   */
  public void setOdometry() {
    poseEstimator.resetPosition(getRotation2d(), swerveModulePositions, getPose());
  }

  public void resetRotation() {
    poseEstimator.resetPosition(
        getRotation2d(),
        swerveModulePositions,
        new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), new Rotation2d()));
  }

  /**
   * Resets the odometer readings using the gyro, SwerveModulePositions (defined in constructor),
   * and Pose2d. Also used in AutonomousScheme.java
   *
   * @param pos the Pose2d to set the odometry
   */
  public void setOdometry(Translation2d translation) {
    poseEstimator.resetPosition(
        getRotation2d(), swerveModulePositions, new Pose2d(translation, getRotation2d()));
  }

  public void setOdometry(Pose2d pose) {
    pigeonGyro.setYaw(pose.getRotation().getDegrees());
    poseEstimator.resetPosition(getRotation2d(), swerveModulePositions, pose);
  }

  public Command setOdometryCommand(Pose2d pose) {
    return runOnce(() -> setOdometry(pose), swerve);
  }

  public void setOdometryAllianceFlip(Pose2d pos) {
    if (Constants.isBlue) return;

    poseEstimator.resetPosition(
        ChoreoAllianceFlipUtil.flip(getRotation2d()),
        swerveModulePositions,
        ChoreoAllianceFlipUtil.flip(pos));
    return;
  }

  /** Gyroscope Methods (Pigeon2) */
  public void zeroHeading() {
    pigeonGyro.reset();
    setOdometry(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(0)));
  }

  /**
   * Method to get the facing direction of the gyro.
   *
   * @return The facing direction of the gyro, between -360 and 360 degrees.
   */
  public double getHeading() {
    return Math.IEEEremainder(pigeonGyro.getYaw().getValueAsDouble(), 360);
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
   * Specifically for the poseEstimator. Anywhere else, get the poseEstimator's estimated heading,
   * or it'll mess things up.
   *
   * @return
   */
  public Rotation2d getRotation2d() {

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

      Logger.recordOutput("gyro/gyroAngleSim", gyroAngleSim);

      previousSwerveModulePositions = currentSwerveModulePositions;

      return Rotation2d.fromRadians(gyroAngleSim);
    }

    Logger.recordOutput("gyro/gyro Rotation2d", pigeonGyro.getRotation2d());
    return pigeonGyro.getRotation2d();

    // .plus(Rotation2d.fromRadians(Math.PI));
  }

  public void setRotation2d(Rotation2d rotation2d) {
    setOdometry(new Pose2d(getPose().getTranslation(), rotation2d));
  }
}

package frc.helpers.maps;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class Constants {

  public static Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // public static DriverStation.Alliance currentAlliance = Alliance.Blue;

  public static boolean isBlue() {
    boolean isBlue = true;
    if (DriverStation.getAlliance().isPresent()) {
      isBlue = DriverStation.getAlliance().get() == Alliance.Blue ? true : false;
    }
    return isBlue;
  }

  public static void setCurrentMode() {
    if (RobotBase.isReal()) {
      currentMode = Mode.REAL;
    } else if (RobotBase.isSimulation()) {
      currentMode = Mode.SIM;
    }
  }

  public static class ConversionConstants {

    public static final double WHEEL_CIRCUMFRENCE = Units.inchesToMeters(4 * Math.PI);

    //  rotations of the turn motor to one rotation of the wheel
    // how much of a rotation the wheel turns for one rotation of the turn motor
    public static final double TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS = 1 / 18.75;

    // How many radians the wheel pivots for one full rotation of the turn motor
    public static final double TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS =
        Units.rotationsToRadians(TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS);

    public static final double TURN_MOTOR_RADIANS_PER_SECOND =
        TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS / 60.0;

    // 5.36 rotations of the drive motor to one spin of the wheel (L3+)
    // 5.9 rotations of the drive motor to one spin of the wheel (L2+)
    public static final double DRIVE_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS = 1.0 / 5.36;
    // horizontal distance travelled by one motor rotation
    public static final double HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION =
        WHEEL_CIRCUMFRENCE * DRIVE_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS;

    public static final double DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR =
        HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION / 60.0;
  }

  public static class MotorConstants {

    // Swerve Drive Base Motor & CAN ID Constants
    public static final int FRONT_RIGHT_DRIVE = 7;
    public static final boolean FRONT_RIGHT_DRIVE_REVERSE = false;
    public static final double FRONT_RIGHT_DRIVE_ENCODER = 1;
    public static final int FRONT_RIGHT_TURN = 8;
    public static final boolean FRONT_RIGHT_TURN_REVERSE = true;
    public static final double FRONT_RIGHT_TURN_ENCODER = 1;

    public static final int FRONT_LEFT_DRIVE = 4;
    public static final boolean FRONT_LEFT_DRIVE_REVERSE = true;
    public static final double FRONT_LEFT_DRIVE_ENCODER = 1;
    public static final int FRONT_LEFT_TURN = 3;
    public static final boolean FRONT_LEFT_TURN_REVERSE = true;
    public static final double FRONT_LEFT_TURN_ENCODER = 1;

    public static final int BACK_RIGHT_DRIVE = 10;
    public static final boolean BACK_RIGHT_DRIVE_REVERSE = false;
    public static final double BACK_RIGHT_DRIVE_ENCODER = 1;
    public static final int BACK_RIGHT_TURN = 11;
    public static final boolean BACK_RIGHT_TURN_REVERSE = true;
    public static final double BACK_RIGHT_TURN_ENCODER = 1;

    public static final int BACK_LEFT_DRIVE = 1;
    public static final boolean BACK_LEFT_DRIVE_REVERSE = true;
    public static final double BACK_LEFT_DRIVE_ENCODER = 1;
    public static final int BACK_LEFT_TURN = 2;
    public static final boolean BACK_LEFT_TURN_REVERSE = true;
    public static final double BACK_LEFT_TURN_ENCODER = 1;

    public static final int CLIMBER = 9;
    public static final boolean CLIMBER_REVERSE = false;

    public static final int[] ELEVATOR = {12, 13};
    public static final boolean[] ELEVATOR_REVERSE = {false, true};

    public static final int ARM = 14;
    public static final boolean ARM_REVERSE = false;

    public static final int WRIST = 15;
    public static final boolean WRIST_REVERSE = false;

    public static final int INTAKE = 16;
    public static final boolean INTAKE_REVERSE = false;

    public static final int ALGAE_WRIST = 5;
    public static final boolean ALGAE_WRIST_REVERSE = false;

    public static final int ALGAE_INTAKE = 6;
    public static final boolean ALGAE_INTAKE_REVERSE = false;

    public static final int PIGEON = 25;
  }

  public static class SwerveConstants {

    // Absolute Encoder Ports
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER = 2;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER = 1;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER = 3;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER = 0;

    public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 2.974;
    public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 4.540;
    public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = 1.960;
    public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = 0.335;

    // Robot Constants (change with SysId)
    // max speed in free sprint: used in getting velocities of swerve modules
    public static final double MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL = 3.71;

    // Velocity Limits
    public static final double MAX_DRIVE_SPEED_METERS_PER_SECOND = 5;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;

    // Rate Limiters (acceleration)
    public static final double DRIVE_RATE_LIMIT = MAX_DRIVE_SPEED_METERS_PER_SECOND * 1.5;
    public static final double TURN_RATE_LIMIT = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    public static final PathConstraints AUTO_PATH_CONSTRAINTS =
        new PathConstraints(
            MAX_DRIVE_SPEED_METERS_PER_SECOND - 2,
            DRIVE_RATE_LIMIT - 0.3,
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
            TURN_RATE_LIMIT);

    public static final RobotConfig ROBOT_CONFIG =
        new RobotConfig(
            100,
            100,
            new ModuleConfig(
                Inches.of(2),
                MetersPerSecond.of(SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL),
                1.0,
                new DCMotor(12, 2.6, 105, 1.8, 594.39, 1),
                Current.ofBaseUnits(90, Amps),
                1),
            SwerveConstants.TRACK_WIDTH);

    public static final TrapezoidProfile.Constraints thetaControlConstraints =
        new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TURN_RATE_LIMIT);

    // Robot Dimensions (relative to wheel locations)
    // Since this robot is a square, no need for 2 values. In a non-square chassis,
    // 2 values needed.

    // Front to back
    public static final double WHEEL_BASE =
        Units.inchesToMeters(24.750000); // from drive shaft to drive shaft. Previous was
    // Right to Left                                                            // 27
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.750000);

    public static final double RADIUS = Math.sqrt(2) * (WHEEL_BASE / 2);

    /** FR FL BR BL. Same as order of swerve module states */
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2));

    public static final Pose2d INITIAL_POSE =
        new Pose2d(0, 0, new Rotation2d(0)); // must be in meters!

    /** This is important! This is the frequency at which odometry data is updated. */
    public static final double ODOMETRY_FREQUENCY = 100;
  }

  public static class ArmConstants {

    // 1/4 * 1/3 * 32/48
    public static final double ARM_MOTOR_ROTATIONS_TO_ARM_ROTATIONS = (.333 * .250 * 0.500);

    // public static final double ARM_MOTOR_ROTATIONS_TO_ARM_ROTATIONS = 1;

    public static final double ARM_MOTOR_ROTATIONS_TO_ARM_ROTATIONS_RADIANS =
        Units.rotationsToRadians(ARM_MOTOR_ROTATIONS_TO_ARM_ROTATIONS);

    public static final double ARM_MOTOR_RADIANS_PER_SECOND_CONVERSION_FACTOR =
        ARM_MOTOR_ROTATIONS_TO_ARM_ROTATIONS_RADIANS / 60;

    public static final double ARM_THROUGHBORE_OFFSET = 0;
  }

  public static class ElevatorConstants {

    public static final double ELEVATOR_MOTOR_ROTATIONS_TO_AXLE_ROTATIONS = 1.0;

    // How many rotations of the main axle to 1 meter of elevation
    public static final double AXLE_ROTATION_TO_HEIGHT_METERS = 1.0;

    public static final double HEIGHT_METERS_PER_ELEVATOR_MOTOR_ROTATIONS =
        ELEVATOR_MOTOR_ROTATIONS_TO_AXLE_ROTATIONS * AXLE_ROTATION_TO_HEIGHT_METERS;

    public static final double ELEVATOR_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR =
        HEIGHT_METERS_PER_ELEVATOR_MOTOR_ROTATIONS / 60;

    public static final double ELEVATOR_THROUGHBORE_OFFSET = 0;
    public static final int HALL_EFFECT_PORT = 0;

    public static double[] elevatorPositions = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    public static double elevatorMaxVelocity = 0;
    public static double elevatorMaxAcceleration = 0;
    public static double elevatorKP = 0;
    public static double elevatorKI = 0;
    public static double elevatorKD = 0;
  }

  public static class WristConstants {

    // 1/4 * 1/3 * 36/48
    public static final double WRIST_MOTOR_ROTATIONS_TO_WRIST_ROTATIONS = (.25 * .333 * .75);

    public static final double WRIST_MOTOR_ROTATIONS_TO_WRIST_ROTATIONS_RADIANS =
        Units.rotationsToRadians(WRIST_MOTOR_ROTATIONS_TO_WRIST_ROTATIONS);

    public static final double WRIST_MOTOR_RADIANS_PER_SECOND_CONVERSION_FACTOR =
        WRIST_MOTOR_ROTATIONS_TO_WRIST_ROTATIONS_RADIANS / 60;

    public static final double WRIST_THROUGHBORE_OFFSET = 0;

    public static boolean WRIST_REVERSE = true;

    // private static final double WRIST_GEAR_RATIO = 1.0;
    // private static final double WRIST_POSITION_COEFFICIENT =
    //     (2 * Math.PI) * (WRIST_GEAR_RATIO * 2048); // idk why we muliply by 2048 but we do
    // private static final double WRIST_VELOCITY_COEFFICIENT = WRIST_POSITION_COEFFICIENT * 10;

    // public static final double WRIST_SLOW_ACCELERATION = Units.degreesToRadians(500);
    // public static final double WRIST_FAST_ACCELERATION = Units.degreesToRadians(750);
    // public static final double WRIST_VELOCITY = Units.degreesToRadians(300);

    // private static final double WRIST_SLOW_ACCELERATION_CONSTRAINT =
    //     WRIST_SLOW_ACCELERATION / WRIST_VELOCITY_COEFFICIENT;
    // private static final double WRIST_FAST_ACCELERATION_CONSTRAINT =
    //     WRIST_FAST_ACCELERATION / WRIST_VELOCITY_COEFFICIENT;
    // private static final double SHOULDER_VELOCITY_CONSTRAINT =
    //     WRIST_VELOCITY / WRIST_VELOCITY_COEFFICIENT;

  }

  public static class IntakeConstants {
    public static final int BEAM_BREAK_PORT = 0;
  }

  public static class AlgaeConstants {}

  public class FeedForwardConstants {

    // TODO Do sysid to get values
    public static final double DRIVE_KS = 0.16681;
    public static final double DRIVE_KV = 2.609;
    public static final double DRIVE_KA = 0.51582;

    public static final double TURNKS = 0;
    public static final double TURNKV = 0;

    // /* TODO SysId these values */

    // KS should be >= 0, so we'll override it to 0.
    public static final double ELEVATOR_KS = 0;
    public static final double ELEVATOR_KV = 2.0129;
    public static final double ELEVATOR_KA = 4.5878;

    // conversion by 0.25 (Not Ideal)
    public static final double ARM_KS = 0.16328;
    public static final double ARM_KV = 0.0029191;
    public static final double ARM_KA = 0.0016397;
    public static final double ARM_KG = 0.15212;

    public static final double WRIST_KS = 0.16328;
    public static final double WRIST_KV = 0.0029191;
    public static final double WRIST_KA = 0.0016397;
    public static final double WRIST_KG = 0.15212;
  }

  public class FieldPositionConstants {

    public static final Transform2d FRONT_REEF_LEFT_OFFSET =
        new Transform2d(Inches.of(0), Inches.of(0), new Rotation2d());

    public static final Transform2d FRONT_REEF_RIGHT_OFFSET =
        new Transform2d(Inches.of(0), Inches.of(0), new Rotation2d());

    public static final Transform2d BACK_REEF_LEFT_OFFSET =
        new Transform2d(Inches.of(0), Inches.of(0), new Rotation2d(Math.PI));
    public static final Transform2d BACK_REEF_RIGHT_OFFSET =
        new Transform2d(Inches.of(0), Inches.of(0), new Rotation2d(Math.PI));

    public static final Transform2d CORAL_STATION_LEFT_OFFSET =
        new Transform2d(Inches.of(0), Inches.of(0), new Rotation2d());
    public static final Transform2d CORAL_STATION_RIGHT_OFFSET =
        new Transform2d(Inches.of(0), Inches.of(0), new Rotation2d());

    public static final Transform2d PROCESSOR_OFFSET =
        new Transform2d(Inches.of(0), Inches.of(0), new Rotation2d());
  }

  /** Back Camera */
  public static class cameraOne {

    public static final String CAMERA_ONE_NAME = "Back Camera";

    // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
    public static final Translation3d ROBOT_TO_CAMERA_TRANS =
        new Translation3d(-0.378716, 0, 0.271038);
    public static final Rotation3d ROBOT_TO_CAMERA_ROT =
        new Rotation3d(0, Math.toRadians(9.369898), Math.PI);

    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(ROBOT_TO_CAMERA_TRANS, ROBOT_TO_CAMERA_ROT);
  }

  public static class cameraTwo {
    public static final String CAMERA_TWO_NAME = "Front Camera";

    public static final Translation3d ROBOT_TO_CAMERA_TRANS =
        new Translation3d(0.344237, 0, 0.177780);

    public static final Rotation3d ROBOT_TO_CAMERA_ROT = new Rotation3d(0, Math.toRadians(0), 0);

    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(ROBOT_TO_CAMERA_TRANS, ROBOT_TO_CAMERA_ROT);
  }

  public static class AprilTags {
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static final Map<Integer, Pose2d> TAG_PROPERTIES =
        APRIL_TAG_FIELD_LAYOUT.getTags().stream()
            .map(tag -> Map.entry(tag.ID, tag.pose.toPose2d()))
            .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));

    public static final int[] TAG_IDS =
        APRIL_TAG_FIELD_LAYOUT.getTags().stream().mapToInt(tag -> tag.ID).toArray();
    public static final List<Pose2d> TAG_POSES =
        APRIL_TAG_FIELD_LAYOUT.getTags().stream()
            .map(tag -> tag.pose.toPose2d())
            .collect(Collectors.toList());

    public static final Map<Integer, Pose2d> TAG_MAP =
        IntStream.range(0, TAG_IDS.length)
            .boxed()
            .collect(Collectors.toMap(i -> TAG_IDS[i], TAG_POSES::get));

    public static final int[] REEF_IDS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    public static final List<Pose2d> REEF_POSES =
        Arrays.stream(REEF_IDS).mapToObj(id -> TAG_PROPERTIES.get(id)).collect(Collectors.toList());

    public static final Map<Integer, Pose2d> REEF_MAP =
        IntStream.range(0, REEF_IDS.length)
            .boxed()
            .collect(Collectors.toMap(i -> REEF_IDS[i], REEF_POSES::get));

    public static final int[] CORAL_STATION_IDS = {1, 2, 12, 13};
    public static final List<Pose2d> CORAL_STATION_POSES =
        Arrays.stream(CORAL_STATION_IDS)
            .mapToObj(id -> TAG_PROPERTIES.get(id))
            .collect(Collectors.toList());

    public static final Map<Integer, Pose2d> CORAL_STATION_MAP =
        IntStream.range(0, CORAL_STATION_IDS.length)
            .boxed()
            .collect(Collectors.toMap(i -> CORAL_STATION_IDS[i], CORAL_STATION_POSES::get));

    public static final int[] PROCESSOR_IDS = {3, 16};
    public static final List<Pose2d> PROCESSOR_POSES =
        Arrays.stream(PROCESSOR_IDS)
            .mapToObj(id -> TAG_PROPERTIES.get(id))
            .collect(Collectors.toList());

    public static final Map<Integer, Pose2d> PROCESSOR_MAP =
        IntStream.range(0, PROCESSOR_IDS.length)
            .boxed()
            .collect(Collectors.toMap(i -> PROCESSOR_IDS[i], PROCESSOR_POSES::get));

    public static final int[] BARGE_IDS = {4, 5, 14, 15};
    public static final List<Pose2d> BARGE_POSES =
        Arrays.stream(BARGE_IDS)
            .mapToObj(id -> TAG_PROPERTIES.get(id))
            .collect(Collectors.toList());

    public static final Map<Integer, Pose2d> BARGE_MAP =
        IntStream.range(0, BARGE_IDS.length)
            .boxed()
            .collect(Collectors.toMap(i -> BARGE_IDS[i], BARGE_POSES::get));
  }

  // safely divide
  public double safeDivision(double numerator, double denominator) {
    if (Math.abs(denominator) < 0.00001) {
      return 0.0;
    }
    double dividend = numerator / denominator;
    return dividend;
  }

  public static int beamBrakePort() {
    return 7; // fill with actual port the beam brake is in
  }
}

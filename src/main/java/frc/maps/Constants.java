package frc.maps;

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

public class Constants {

  // Might be more specific than this.
  public static final double WHEEL_CIRCUMFRENCE = Units.inchesToMeters(4 * Math.PI);

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

  public static void getCurrentMode() {
    if (RobotBase.isReal()) {
      currentMode = Mode.REAL;
    } else if (RobotBase.isSimulation()) {
      currentMode = Mode.SIM;
    }
  }

  public static class ConversionConstants {

    // 150/7 rotations of the turn motor to one rotation of the wheel
    // how much of a rotation the wheel turns for one rotation of the turn motor
    public static final double TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS = 7.0 / 150.0;

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

    /** Wrist */
  }

  public static class MotorConstants {

    // Swerve Drive Base Motor & CAN ID Constants
    public static final int FRONT_RIGHT_DRIVE = 3;
    public static final boolean FRONT_RIGHT_DRIVE_REVERSE = true;
    public static final double FRONT_RIGHT_DRIVE_ENCODER = 1;
    public static final int FRONT_RIGHT_TURN = 4;
    public static final boolean FRONT_RIGHT_TURN_REVERSE = true;
    public static final double FRONT_RIGHT_TURN_ENCODER = 1;

    public static final int FRONT_LEFT_DRIVE = 7;
    public static final boolean FRONT_LEFT_DRIVE_REVERSE = true;
    public static final double FRONT_LEFT_DRIVE_ENCODER = 1;
    public static final int FRONT_LEFT_TURN = 6;
    public static final boolean FRONT_LEFT_TURN_REVERSE = true;
    public static final double FRONT_LEFT_TURN_ENCODER = 1;

    public static final int BACK_RIGHT_DRIVE = 2;
    public static final boolean BACK_RIGHT_DRIVE_REVERSE = true;
    public static final double BACK_RIGHT_DRIVE_ENCODER = 1;
    public static final int BACK_RIGHT_TURN = 1;
    public static final boolean BACK_RIGHT_TURN_REVERSE = true;
    public static final double BACK_RIGHT_TURN_ENCODER = 1;

    public static final int BACK_LEFT_DRIVE = 9;
    public static final boolean BACK_LEFT_DRIVE_REVERSE = true;
    public static final double BACK_LEFT_DRIVE_ENCODER = 1;
    public static final int BACK_LEFT_TURN = 8;
    public static final boolean BACK_LEFT_TURN_REVERSE = true;
    public static final double BACK_LEFT_TURN_ENCODER = 1;

    public static final int ALGAE_WRIST = 10;
    public static final boolean ALGAE_WRIST_REVERSE = false;

    public static final int ALGAE_INTAKE = 11;
    public static final boolean ALGAE_INTAKE_REVERSE = false;

    public static final int CLIMBER = 12;
    public static final boolean CLIMBER_REVERSE = false;

    public static final int[] ELEVATOR = {13, 14};
    public static final boolean[] ELEVATOR_REVERSE = {false, true};

    public static final int ARM = 15;
    public static final boolean ARM_REVERSE = false;

    public static final int[] INTAKE = {16, 17};
    public static final boolean[] INTAKE_REVERSE = {false, true};

    public static final int WRIST = 18;
  }

  public static class SwerveConstants {

    // Absolute Encoder Ports
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER = 1;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER = 3;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER = 0;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER = 2;

    public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 2.348;
    public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 5.6393;
    public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = 1.7729;
    public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = 5.965215;

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
        Units.inchesToMeters(24.7500000000); // from drive shaft to drive shaft. Previous was
    // Right to Left                                                            // 27
    public static final double TRACK_WIDTH = Units.inchesToMeters(24.750000);

    public static final double RADIUS = Math.sqrt(2) * (WHEEL_BASE / 2);

    /** FR FL BR BL. Same as order of swerve module states */
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2));

    public static Pose2d INITIAL_POSE = new Pose2d(0, 0, new Rotation2d(0)); // must be in meters!

    /** This is important! This is the frequency at which odometry data is updated. */
    public static final double ODOMETRY_FREQUENCY = 100;
  }

  public static class ArmConstants {

    public static final double ARM_MOTOR_ROTATIONS_TO_ARM_ROTATIONS = 1.0;

    public static final double ARM_MOTOR_ROTATIONS_TO_ARM_ROTATIONS_RADIANS =
        Units.rotationsToRadians(ARM_MOTOR_ROTATIONS_TO_ARM_ROTATIONS);

    public static final double ARM_MOTOR_RADIANS_PER_SECOND_CONVERSION_FACTOR =
        ARM_MOTOR_ROTATIONS_TO_ARM_ROTATIONS_RADIANS / 60;

    public static final double ARM_THROUGHBORE_OFFSET = 1.0;
  }

  public static class ElevatorConstants {

    public static final double ELEVATOR_MOTOR_ROTATIONS_TO_AXLE_ROTATIONS = 1.0;

    // How many rotations of the main axle to 1 meter of elevation
    public static final double AXLE_ROTATION_TO_HEIGHT_METERS = 1.0;

    public static final double HEIGHT_METERS_PER_ELEVATOR_MOTOR_ROTATIONS =
        ELEVATOR_MOTOR_ROTATIONS_TO_AXLE_ROTATIONS * AXLE_ROTATION_TO_HEIGHT_METERS;

    public static final double ELEVATOR_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR =
        HEIGHT_METERS_PER_ELEVATOR_MOTOR_ROTATIONS / 60;

    public static final double ELEVATOR_THROUGHBORE_OFFSET = 1.0;

    public static double[] elevatorPositions = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    public static double elevatorMaxVelocity = 90;
    public static double elevatorMaxAcceleration = 90;
    public static double elevatorKP = 90;
    public static double elevatorKI = 90;
    public static double elevatorKD = 90;
  }

  public static class WristConstants {

    public static final double WRIST_MOTOR_ROTATIONS_TO_WRIST_ROTATIONS = 1.0;

    public static final double WRIST_MOTOR_ROTATIONS_TO_WRIST_ROTATIONS_RADIANS =
        Units.rotationsToRadians(WRIST_MOTOR_ROTATIONS_TO_WRIST_ROTATIONS);

    public static final double WRIST_MOTOR_RADIANS_PER_SECOND_CONVERSION_FACTOR =
        WRIST_MOTOR_ROTATIONS_TO_WRIST_ROTATIONS_RADIANS / 60;

    public static final double WRIST_THROUGHBORE_OFFSET = 1.0;

    public static boolean WRIST_REVERSE = false;
  }

  public static class algaeConstants {}

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

  public class FieldPositionConstants {}

  public class MechanismPositions {
    public static double ELEVATOR_INTAKE = 0;
    public static double WRIST_INTAKE = 0;

    public static double ELEVATOR_SHOOT = 0;
    public static double WRIST_SHOOT = 15.619040489196777;
    // public static double WRIST_SHOOT = 8.714315414428711;

    public static double ELEVATOR_AMP = 77;
    public static double WRIST_AMP = 50.5;

    public static double ELEVATOR_HUMAN_PLAYER = 0;
    public static double WRIST_HUMAN_PLAYER = 0;

    public static double ELEVATOR_TOP = 75;

    public static double WRIST_TRAVEL = 20;
  }

  public class XboxConstants {
    // Joystick Axises
    public static final int L_JOYSTICK_HORIZONTAL = 0;
    public static final int L_JOYSTICK_VERTICAL = 1;
    public static final int LT = 2;
    public static final int RT = 3;
    public static final int R_JOYSTICK_HORIZONTAL = 4;
    public static final int R_JOYSTICK_VERTICAL = 5;

    // Controller Buttons
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LB_BUTTON = 5;
    public static final int RB_BUTTON = 6;
    public static final int SELECT_BUTTON = 7;
    public static final int START_BUTTON = 8;

    // These buttons are when you push down the left and right circle pad
    public static final int L_JOYSTICK_BUTTON = 9;
    public static final int R_JOYSTICK_BUTTON = 10;

    // D Pad Buttons
    public static final int DPAD_UP = 0;
    public static final int DPAD_UP_RIGHT = 45;
    public static final int DPAD_RIGHT = 90;
    public static final int DPAD_DOWN_RIGHT = 135;
    public static final int DPAD_DOWN = 180;
    public static final int DPAD_DOWN_LEFT = 225;
    public static final int DPAD_LEFT = 270;
    public static final int DPAD_UP_LEFT = 315;

    // Controller Zeroes
    public static final double ZERO = 0.15;
  }

  public static class cameraOne {
    public static final String CAMERA_ONE_NAME = "FrontCamera";
    public static final int CAMERA_ONE_PIPELINE = 9;
    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(
            new Translation3d(Inches.of(9.3418), Inches.of(0), Inches.of(14.157)),
            new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0)));
  }

  public static class cameraTwo {
    public static final String CAMERA_TWO_NAME = "BackCamera";
    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(
            new Translation3d(Inches.of(9.3418), Inches.of(0), Inches.of(14.157)),
            new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0)));
  }

  /**
   * Gotten from here
   * https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
   */
  public static class AprilTags {
    public static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // public static int BLUE_SOURCE_LEFT = 1;
    // public static int BLUE_SOURCE_RIGHT = 2;
    // public static int RED_SPEAKER_BOTTOM = 3;
    // public static int RED_SPEAKER_TOP = 4;
    // public static int RED_AMP = 5;
    // public static int BLUE_AMP = 6;
    // public static int BLUE_SPEAKER_TOP = 7;
    // public static int BLUE_SPEAKER_BUTTON = 8;
    // public static int RED_SOURCE_LEFT = 9;
    // public static int RED_SOURCE_RIGHT = 10;
    // public static int RED_STAGE_BOTTOM = 11;
    // public static int RED_STAGE_TOP = 12;
    // public static int RED_STAGE_SIDE = 13;
    // public static int BLUE_STAGE_SIDE = 14;
    // public static int BLUE_STAGE_TOP = 15;
    // public static int BLUE_STAGE_BOTTOM = 16;
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
    return 0; // fill with actual port the beam brake is in
  }
}

// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  public static class MotorConstants {
    public static final int ELEVATOR_ONE = 1;
    public static final boolean ELEVATOR_ONE_REVERSE = false;
    public static final int ELEVATOR_TWO = 2;
    public static final boolean ELEVATOR_TWO_REVERSE = true;
    public static final int CORAL_WRIST = 3;
    public static final boolean CORAL_WRIST_REVERSE = true;
    
  }

  public static class ElevatorConstants{
    double[] elevatorHeights = {0,1,2,3,4,5,6,7};
    double elevatorMaxVelocity = 90;
    double elevatorMaxAcceleration = 90;
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

    // 6.75 rotations of the drive motor to one spin of the wheel
    public static final double DRIVE_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS = 1.0 / 6.75;
    // horizontal distance travelled by one motor rotation
    public static final double HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION =
        WHEEL_CIRCUMFRENCE * DRIVE_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS;

    public static final double DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR =
        HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION / 60.0;
  }
}

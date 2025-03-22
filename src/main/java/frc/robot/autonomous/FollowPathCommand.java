// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.util.BlinkinLEDController;
import frc.util.BlinkinLEDController.BlinkinPattern;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class FollowPathCommand extends Command {

  private Timer timer = new Timer();

  private PathPlannerTrajectory trajectory;
  private SwerveDriveSubsystem swerve;

  private Pose2d currentPose;

  // private PathPlannerTrajectoryState lastState, wantedState;

  private static PIDController translationPID;

  private static ProfiledPIDController xPID;
  private static ProfiledPIDController yPID;

  private static ProfiledPIDController rotationPID;

  static {
    translationPID = new PIDController(1, 0, 0);

    xPID = new ProfiledPIDController(1, 0, 0, new Constraints(.015, 2));
    yPID = new ProfiledPIDController(1, 0, 0, new Constraints(.015, 2));

    rotationPID = new ProfiledPIDController(1, 0, 0, new Constraints(2, 3));
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  private DoubleSupplier driveSpeedModifier = () -> 0;

  /**
   * Follows a PathPlannerTrajectory
   *
   * @param trajectory the PathPlannerTrajectory
   */
  public FollowPathCommand(PathPlannerTrajectory trajectory, SwerveDriveSubsystem swerve) {
    this.swerve = swerve;

    this.trajectory = trajectory;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    BlinkinLEDController.getInstance().setPattern(BlinkinPattern.VIOLET);

    currentPose = RobotState.getInstance().getPose();

    timer.reset();
    timer.start();

    // lastState = trajectory.getInitialState();

    translationPID.reset();

    xPID.reset(currentPose.getX());
    yPID.reset(currentPose.getY());

    rotationPID.reset(RobotState.getInstance().getPoseAngleRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // BlinkinLEDController.getInstance().setIfNotAlready(BlinkinPattern.BREATH_RED);

    currentPose = RobotState.getInstance().getPose();

    /** The current time. */
    double currentTime = timer.get();

    /** Get the state of the robot at this current time in the path. */
    PathPlannerTrajectoryState wantedState = trajectory.sample(currentTime);

    // wantedState.pose = Constants.isBlue() ? wantedState.pose : wantedState.flip().pose;

    // if (!Constants.isBlue) wantedState.pose = wantedState.flip().pose;
    // wantedState.heading = wantedState.pose.getRotation();

    // double xSpeed =
    //     wantedState.linearVelocity
    //         * Math.cos(wantedState.heading.getRadians())
    //         * driveSpeedModifier.getAsDouble();
    // double ySpeed =
    //     wantedState.linearVelocity
    //         * Math.sin(wantedState.heading.getRadians())
    //         * driveSpeedModifier.getAsDouble();

    // Logger.recordOutput("FollowPathCommand/wantedChoreoVelocityX", xSpeed);
    // Logger.recordOutput("FollowPathCommand/wantedChoreoVelocityY", ySpeed);

    // double xPIDOutput = translationPID.calculate(currentPose.getX(), wantedState.pose.getX());
    // double yPIDOutput = translationPID.calculate(currentPose.getY(), wantedState.pose.getY());

    // Logger.recordOutput("FollowPathCommand/x PID Output", xPIDOutput);
    // Logger.recordOutput("FollowPathCommand/y PID Output", yPIDOutput);

    // double wantedRotationSpeeds =
    //     rotationPID.calculate(
    //         currentPose.getRotation().getRadians(), wantedState.heading.getRadians());

    // xSpeed = xSpeed + xPIDOutput;
    // ySpeed = ySpeed + yPIDOutput;

    // xSpeed =
    //     clamp(
    //         xSpeed,
    //         -Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL,
    //         Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL);
    // ySpeed =
    //     clamp(
    //         ySpeed,
    //         -Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL,
    //         Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL);

    // wantedRotationSpeeds =
    //     clamp(
    //         wantedRotationSpeeds,
    //         -Constants.SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
    //         Constants.SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);
    // // double setXSpeed = xSpeed + xPID;
    // // double setYSpeed = ySpeed + yPID;

    // // setXSpeed = xRateLimiter.calculate(setXSpeed);
    // //               ySpeed = yRateLimiter.calculate(ySpeed);

    // Logger.recordOutput("xSpeed + xPID", xSpeed);
    // Logger.recordOutput("ySpeed + yPID", ySpeed);
    // Logger.recordOutput("wantedRotationSpeeds", wantedRotationSpeeds);

    /** Add alliance transform! */

    /** Create a ChassisSpeeds object to represent how the robot should be moving at this time. */
    ChassisSpeeds chassisSpeeds =
        // ChassisSpeeds.fromFieldRelativeSpeeds(
        //     xSpeed, ySpeed, wantedRotationSpeeds, RobotState.getInstance().getPoseRotation2d());
        swerve.swerveFollower.calculateRobotRelativeSpeeds(
            RobotState.getInstance().getPose(), wantedState);

    Logger.recordOutput("FollowPathCommand/wantedAutoPose", wantedState.pose);

    swerve.driveRobotRelative(chassisSpeeds);
    // SwerveDrive.getInstance().setModuleStates(moduleStates);
    // Logger.recordOutput("Autonomous Set moduleStates", moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.timer.stop();
    swerve.stopModules();

    BlinkinLEDController.getInstance().setPattern(BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double translationError =
        Math.abs(
            trajectory
                .getEndState()
                .pose
                .getTranslation()
                .getDistance(RobotState.getInstance().getPose().getTranslation()));
    double rotationError =
        Math.abs(
            RobotState.getInstance().getPoseAngleDegrees()
                - trajectory.getEndState().heading.getDegrees());

    Logger.recordOutput("FollowPathCommand/translationError", translationError);
    Logger.recordOutput("FollowPathCommand/rotationError", rotationError);

    // return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    return (translationError < 1 && rotationError < 5) || timer.hasElapsed(4);
  }

  public static double clamp(double measurement, double min, double max) {
    if (measurement < min) return min;
    if (measurement > max) return max;
    return measurement;
  }
}

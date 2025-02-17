// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.helpers.maps.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

public class FollowPathCommand extends Command {

  Timer timer = new Timer();

  PathPlannerTrajectory trajectory;
  SwerveDriveSubsystem swerve;

  Pose2d currentPose;

  PathPlannerTrajectoryState lastState, wantedState;

  PIDController translationPID;
  PIDController rotationPID;

  /**
   * Follows a PathPlannerTrajectory
   *
   * @param trajectory the PathPlannerTrajectory
   */
  public FollowPathCommand(PathPlannerTrajectory trajectory, SwerveDriveSubsystem swerve) {
    this.swerve = swerve;
    translationPID = new PIDController(5, 0, 0);
    rotationPID = new PIDController(5, 0, 0);
    rotationPID.enableContinuousInput(0, Math.PI);

    this.trajectory = trajectory;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    rotationPID.reset();
    translationPID.reset();

    timer.reset();
    timer.start();

    lastState = trajectory.getInitialState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPose = RobotState.getInstance().getPose();

    /** The current time. */
    double currentTime = timer.get();

    /** Get the state of the robot at this current time in the path. */
    PathPlannerTrajectoryState wantedState = trajectory.sample(currentTime);

    Pose2d wantedPose = Constants.isBlue() ? wantedState.pose : wantedState.flip().pose;
    Rotation2d wantedHeading = wantedState.heading;

    double xSpeed = wantedState.linearVelocity * Math.cos(wantedHeading.getRadians());
    double ySpeed = wantedState.linearVelocity * Math.sin(wantedHeading.getRadians());

    double xPID = translationPID.calculate(currentPose.getX(), wantedState.pose.getX());
    double yPID = translationPID.calculate(currentPose.getY(), wantedState.pose.getY());

    double wantedRotationSpeeds =
        rotationPID.calculate(currentPose.getRotation().getRadians(), wantedHeading.getRadians());

    Logger.recordOutput("xSpeed + xPID", xSpeed + xPID);
    Logger.recordOutput("ySpeed + yPID", ySpeed + yPID);
    Logger.recordOutput("wantedRotationSpeeds", wantedRotationSpeeds);

    /** Add alliance transform! */

    /** Create a ChassisSpeeds object to represent how the robot should be moving at this time. */
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed + xPID,
            ySpeed + yPID,
            wantedRotationSpeeds,
            RobotState.getInstance().getPoseRotation2d());
    // SwerveDrive.getInstance()
    //     .swerveFollower
    //     .calculateRobotRelativeSpeeds(RobotState.getInstance().currentPose, wantedState);

    Logger.recordOutput("wantedAutoPose", wantedState.pose);

    swerve.driveRobotRelative(chassisSpeeds);
    // SwerveDrive.getInstance().setModuleStates(moduleStates);
    // Logger.recordOutput("Autonomous Set moduleStates", moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.timer.stop();
    swerve.stopModules();
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

    return timer.hasElapsed(trajectory.getTotalTimeSeconds())
        && translationError < 1
        && rotationError < 1;

    // return timer.hasElapsed(20);
  }
}

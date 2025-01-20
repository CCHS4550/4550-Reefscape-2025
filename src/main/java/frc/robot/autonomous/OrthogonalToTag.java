// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.maps.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class OrthogonalToTag extends Command {

  Optional<PhotonTrackedTarget> target;
  Rotation2d targetAngle;
  Pose2d currentPose;
  PathPlannerTrajectoryState targetState;

  SwerveDrivePoseEstimator poseRelativeToTargetEstimator;

  Timer timer = new Timer();

  // If you want to troubleshoot this, you should log the poses and check it in advantagescope or
  // whatever.

  /**
   * This command should rotate the robot such that it is orthogonal to the AprilTag in its vision.
   * UNTESTED
   */
  public OrthogonalToTag() {

    // Only set once, so the target isn't updated.

    /**
     * Quick explanation if any of y'all are curious: getBestCameraToTarget() returns a Transform3d
     * object, that represents the transformation vector you can apply to the camera which will
     * become the camera position. We can apply the .getRotation() decorator to get the Rotation3d,
     * but we only need Rotation2d so we can apply the .toRotation2d decorator, and then get it in
     * degrees with .getDegrees(). So if we think about this as the angle between the 2 poses, it
     * should look somewhat like the picture I attached. So, subtracting 90 degrees and getting the
     * complementary angle would get us the right angle. Stick on a negative cause it should be
     * negative according to that coordinate system and I'm just going to assume that this is common
     * for all cases.
     */
    target =
        Optional.of(
            frc.helpers.vision.PhotonVision.getInstance()
                .frontCamera
                .getLatestResult()
                .getBestTarget());
    targetAngle =
        Rotation2d.fromDegrees(
            -target.get().getBestCameraToTarget().getRotation().toRotation2d().getDegrees() - 180);

    if (target.isPresent()) {
      /** Initialize a temporary PoseEstimator that lasts for this command's length. It will */
      poseRelativeToTargetEstimator =
          new SwerveDrivePoseEstimator(
              Constants.SwerveConstants.DRIVE_KINEMATICS,
              RobotState.getInstance().getPoseRotation2d(),
              RobotState.getInstance().swerveModulePositions,
              // This line below should set the initial pose to (0,0) with an angle of the angle of
              // the AprilTag.

              // If theres a problem, I suspect it's probably here or wherever else getYaw() is
              // called to define the robots pose.
              // It might be negative but theoretically it shouldn't based on the way that
              // Photonvision creates a coordinate system from the Apriltag.
              // I subtracted it from 90 because if you really really really think about it then you
              // need the complementary angle?
              new Pose2d(0, 0, targetAngle));
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveDriveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer.start();

    if (target.isPresent()) {
      currentPose = poseRelativeToTargetEstimator.getEstimatedPosition();
      targetState = new PathPlannerTrajectoryState();

      //  Set target state to (0,0) so it shouldn't move around.
      targetState.pose = new Pose2d();
      // Set the target rotation to 0.
      targetState.heading = Rotation2d.fromDegrees(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Logger.recordOutput("AprilTag Yaw", target.get().getYaw());
    Logger.recordOutput("AprilTag Yaw 2", targetAngle);
    double currentTime = timer.get();

    // I'm not sure if we need to do all this, but it should make it relatively more accurate. By
    // doing this, we are combining swerve drive odometry with vision.
    // Lowkey vision should be accurate enough on its own but there should be no harm in adding more
    // data, since its all run through a Kalman Filter (google it!)
    // Actually now that I think about this, this is lowkey useless since the target is only defined
    // once. We should only rely on swerve drive odometry.
    // poseRelativeToTargetEstimator.addVisionMeasurement(new Pose2d(new Translation2d(),
    // Rotation2d.fromDegrees(target.get().getYaw())), currentTime);

    // This is another place that might be a problem, as this is where the odometry is updated. We
    // have typically always used getRotation2dNegative() which has always worked fine.
    // I feel like this doesn't make much sense though but I'm a little scared to change it. If it
    // works, it works.
    poseRelativeToTargetEstimator.updateWithTime(
        currentTime,
        RobotState.getInstance().getPoseRotation2d(),
        RobotState.getInstance().swerveModulePositions);

    // We only want the angle, not the translation because we only want to rotate, so we set the
    // pose to (0,0) with the angle of the AprilTag.
    currentPose =
        new Pose2d(
            new Translation2d(),
            poseRelativeToTargetEstimator.getEstimatedPosition().getRotation());

    ChassisSpeeds chassisSpeeds =
        SwerveDriveSubsystem.getInstance()
            .swerveFollower
            // .calculateRobotRelativeSpeeds(new Pose2d(0, 0,
            // RobotState.getInstance().getAngleBetweenCurrentAndTargetPose(new Pose2d(0,
            // 0,Rotation2d.fromDegrees(target.getYaw())))), new State());
            .calculateRobotRelativeSpeeds(currentPose, targetState);
    // Convert chassis speeds to individual module states

    SwerveDriveSubsystem.getInstance().driveRobotRelative(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(
            PhotonUtils.getYawToPose(
                    currentPose, new Pose2d(new Translation2d(), targetState.heading))
                .getDegrees())
        < 2);
    // || target.isEmpty();
  }
}

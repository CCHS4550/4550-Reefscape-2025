// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.helpers.vision.*;
import frc.maps.Constants;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class OrthogonalToTag extends Command {

  List<PhotonTrackedTarget> targets;
  Rotation2d targetAngle;
  double targetX;
  double targetY;

  double distanceFromTarget;

  Pose2d currentRelativePose;
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
    targetAngle = getAverageAngle(getTransform3dList());
    targetX = getAverageX(getTransform3dList());
    targetY = getAverageY(getTransform3dList());

    // if (targets.isPresent()) {
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

            new Pose2d(0, 0, new Rotation2d()));

    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(SwerveDriveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer.start();

    // if (target.isPresent()) {
    currentRelativePose = poseRelativeToTargetEstimator.getEstimatedPosition();
    targetState = new PathPlannerTrajectoryState();

    targetState.pose = new Pose2d(targetX, targetY, targetAngle);
    targetState.heading = targetAngle;
  }
  // }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetAngle = getAverageAngle(getTransform3dList());
    targetX = getAverageX(getTransform3dList());
    targetY = getAverageY(getTransform3dList());

    targetState.pose = new Pose2d(targetX, targetY, targetAngle);

    targetState.heading = targetAngle;

    for (int i = 0; i < getTransform3dList().size(); i++) {
      Logger.recordOutput(
          "OrthogonalToTag/Transformations/" + i + "/Translations",
          getTransform3dList().get(i).getTranslation());
      Logger.recordOutput(
          "OrthogonalToTag/Transformations/" + i + "/Rotations",
          getTransform3dList().get(i).getTranslation());
    }

    Pose2d currentGlobalPose =
        RobotState.getInstance()
            .getPose()
            .plus(new Transform2d(currentRelativePose, targetState.pose));
    Logger.recordOutput(
        "OrthogonalToTag/currentGlobalPose",
        new Pose2d(
            currentGlobalPose.getX(), currentGlobalPose.getY(), currentGlobalPose.getRotation()));

    Logger.recordOutput("OrthogonalToTag/ExecutingCommand...", true);

    Logger.recordOutput("OrthogonalToTag/targetPose", targetState.pose);
    Logger.recordOutput(
        "OrthogonalToTag/currentPose", poseRelativeToTargetEstimator.getEstimatedPosition());

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
    for (int i = 0; i < RobotState.getInstance().sampleCount; i++) {
      poseRelativeToTargetEstimator.updateWithTime(
          RobotState.getInstance().sampleTimestamps[i],
          RobotState.getInstance().gyroAngle[i],
          RobotState.getInstance().swerveModulePositionsArray[i]);
    }

    // We only want the angle, not the translation because we only want to rotate, so we set the
    // pose to (0,0) with the angle of the AprilTag.
    currentRelativePose = poseRelativeToTargetEstimator.getEstimatedPosition();

    // ChassisSpeeds chassisSpeeds =
    //     SwerveDriveSubsystem.getInstance()
    //         .swerveFollower
    //         // .calculateRobotRelativeSpeeds(new Pose2d(0, 0,
    //         // RobotState.getInstance().getAngleBetweenCurrentAndTargetPose(new Pose2d(0,
    //         // 0,Rotation2d.fromDegrees(target.getYaw())))), new State());
    //         .calculateRobotRelativeSpeeds(currentRelativePose, targetState);
    // // Convert chassis speeds to individual module states

    // SwerveDriveSubsystem.getInstance().driveRobotRelative(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("OrthogonalToTag/ExecutingCommand...", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(.5);
    // || target.isEmpty();
  }

  /** Helper Methods */
  public double getAverageX(List<Transform3d> transform3dList) {

    List<Distance> distanceList = new ArrayList<>();

    for (Transform3d transform3d : transform3dList) {
      Distance orthogonalAngle = transform3d.getMeasureX();
      distanceList.add(orthogonalAngle);
    }

    int distanceCount = distanceList.size();

    double totalDistance = 0;

    for (int i = 0; i < distanceCount; i++) {
      totalDistance += distanceList.get(i).in(Meter);
    }
    return (totalDistance / distanceCount);
  }

  public double getAverageY(List<Transform3d> transform3dList) {

    List<Distance> distanceList = new ArrayList<>();

    for (Transform3d transform3d : transform3dList) {
      Distance orthogonalAngle = transform3d.getMeasureY();
      distanceList.add(orthogonalAngle);
    }

    int distanceCount = distanceList.size();

    double totalDistance = 0;

    for (int i = 0; i < distanceCount; i++) {
      totalDistance += distanceList.get(i).in(Meter);
    }
    return (totalDistance / distanceCount);
  }

  public Rotation2d getAverageAngle(List<Transform3d> transform3dList) {

    List<Rotation2d> angleList = new ArrayList<>();

    for (Transform3d transform3d : transform3dList) {
      Rotation2d orthogonalAngle =
          Rotation2d.fromDegrees(transform3d.getRotation().toRotation2d().getDegrees() - 180);
      angleList.add(orthogonalAngle);
    }

    int angleCount = angleList.size();

    double totalAngle = 0;

    for (int i = 0; i < angleCount; i++) {
      totalAngle += angleList.get(i).getDegrees();
    }
    return Rotation2d.fromDegrees(totalAngle / angleCount);
  }

  public List<Transform3d> getTransform3dList() {

    // if (RobotBase.isSimulation()) return PhotonVisionSim.getInstance().getTransformListSim();
    // System.out.println(PhotonVisionSim.getInstance().condensedResults.size());
    if (RobotBase.isSimulation())
      return PhotonVisionSim.getInstance().condensedResults.stream()
          .map(
              result ->
                  result.getValue().hasTargets()
                      ? result
                          .getKey()
                          .getRobotToCameraTransform()
                          .plus(result.getValue().getBestTarget().getBestCameraToTarget())
                      : new Transform3d())
          .collect(Collectors.toList());

    if (RobotBase.isReal())
      return PhotonVisionAprilTag.getInstance().condensedResults.stream()
          .map(
              result ->
                  result.getValue().hasTargets()
                      ? result
                          .getKey()
                          .getRobotToCameraTransform()
                          .plus(result.getValue().getBestTarget().getBestCameraToTarget())
                      : new Transform3d())
          .collect(Collectors.toList());

    return null;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.helpers.maps.Constants;
import frc.helpers.vision.*;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

public class OrthogonalToTag extends Command {

  private boolean poseIsNull = true;
  private boolean exitCommand = false;

  private double distanceMetersErr;
  private double angleDegreesErr;

  private SwerveDriveSubsystem swerve;
  private VisionIO vision;

  private int focusedTag;

  private Pose2d currentRelativePose;
  private PathPlannerTrajectoryState targetState;
  private Transform2d transformation;
  private List<Pose2d> idList;

  private Pose2d globalTargetPose;
  private Pose2d globalCurrentPose;

  private SwerveDrivePoseEstimator poseRelativeToTargetEstimator;

  @SuppressWarnings("unused")
  private PIDController translationPID;

  @SuppressWarnings("unused")
  private PIDController rotationPID;

  Timer timer = new Timer();

  // If you want to troubleshoot this, you should log the poses and check it in advantagescope or
  // whatever.

  /**
   * This command should rotate the robot such that it is orthogonal to the AprilTag in its vision.
   * UNTESTED
   */
  public OrthogonalToTag(
      Transform2d transformation,
      List<Pose2d> idList,
      boolean useBestTag,
      SwerveDriveSubsystem swerve,
      VisionIO vision) {
    this.transformation = transformation;
    this.idList = idList;

    this.swerve = swerve;
    this.vision = vision;

    translationPID = new PIDController(5, 0, 0);
    rotationPID = new PIDController(5, 0, 0);

    if (useBestTag) this.focusedTag = RobotState.getInstance().visionInputs.focusedId;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    exitCommand = false;

    poseRelativeToTargetEstimator =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.DRIVE_KINEMATICS,
            RobotState.getInstance().getPoseRotation2d(),
            RobotState.getInstance().swerveModulePositions,
            new Pose2d(0, 0, new Rotation2d()));

    Pose2d samplePose =
        RobotState.getInstance().getPose().nearest(idList).nearest(Constants.AprilTags.TAG_POSES);

    for (int i = 0; i < Constants.AprilTags.TAG_POSES.size(); i++) {
      if (samplePose.equals(Constants.AprilTags.TAG_POSES.get(i)))
        focusedTag = Constants.AprilTags.TAG_IDS[i];
    }

    timer.reset();
    timer.start();

    currentRelativePose = poseRelativeToTargetEstimator.getEstimatedPosition();

    targetState = new PathPlannerTrajectoryState();

    targetState.pose = null;
    targetState.heading = null;

    poseIsNull = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if pose is null, it should get a pose from a thigsndfkhgnldgfsfdfakd

    for (int i = 0; i < getTransform3dList().size(); i++) {
      Logger.recordOutput("OrthogonalToTag/TransformList", getTransform3dList().get(i));
    }
    Logger.recordOutput("OrthogonalToTag/TransformList.size()", getTransform3dList().size());
    Logger.recordOutput("OrthogonalToTag/focusedTag", focusedTag);
    Logger.recordOutput("OrthogonalToTag/ExecutingCommand...", true);

    currentRelativePose = poseRelativeToTargetEstimator.getEstimatedPosition();
    Logger.recordOutput("OrthogonalToTag/currentPose", currentRelativePose);

    /** Update Target Pose */
    if (getTransform3dList().size() > 0) {

      // vision
      //     .getPipelineResults()
      //     .forEach(
      //         result -> {
      //           result
      //               .getValue()
      //               .getTargets()
      //               .forEach(
      //                   target -> {
      //                     poseRelativeToTargetEstimator.addVisionMeasurement(
      //                         new Pose3d(0, 0, 0, new Rotation3d())
      //                             .plus(
      //                                 result
      //                                     .getKey()
      //                                     .getRobotToCameraTransform()
      //                                     .plus(target.getBestCameraToTarget()))
      //                             .toPose2d(),
      //                         result.getValue().getTimestampSeconds());
      //                   });
      //         });

      Rotation2d targetAngle = getAverageAngle(getTransform3dList());
      double targetX = getAverageX(getTransform3dList());
      double targetY = getAverageY(getTransform3dList());

      Logger.recordOutput("OrthogonalToTag/targetAngle", targetAngle);
      Logger.recordOutput("OrthogonalToTag/targetX", targetX);
      Logger.recordOutput("OrthogonalToTag/targetY", targetY);

      targetState.pose =
          new Pose2d(targetX, targetY, targetAngle)
              .plus(new Transform2d(new Pose2d(), currentRelativePose));
      targetState.pose = targetState.pose.plus(transformation);
      targetState.heading = targetState.pose.getRotation();
    }

    if (targetState.pose == null) {
      targetState.pose =
          new Pose2d(
              Constants.AprilTags.TAG_MAP
                  .get(focusedTag)
                  .plus(transformation)
                  .getTranslation()
                  .minus(RobotState.getInstance().getPose().getTranslation()),
              Constants.AprilTags.TAG_MAP.get(focusedTag).plus(transformation).getRotation());
      targetState.heading =
          Constants.AprilTags.TAG_MAP.get(focusedTag).plus(transformation).getRotation();
    }

    Logger.recordOutput("OrthogonalToTag/targetPose", targetState.pose);

    /** Update Odometry Pose Estimation */
    if (RobotState.getInstance().visionInputs.hasTarget == false) {
      if (RobotState.getInstance().sampleCountHF > 0
          && RobotState.getInstance().pigeonGyro.isConnected()
          && RobotState.getInstance().useHF) {
        for (int i = 0; i < RobotState.getInstance().sampleCountHF; i++) {
          poseRelativeToTargetEstimator.updateWithTime(
              RobotState.getInstance().sampleTimestampsHF[i],
              RobotState.getInstance().gyroAnglesHF[i],
              RobotState.getInstance().swerveModulePositionsHF[i]);

          Logger.recordOutput("OrthogonalToTag/Using HF", true);
        }
      } else {

        poseRelativeToTargetEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            RobotState.getInstance().getPoseRotation2d(),
            RobotState.getInstance().swerveModulePositions);

        Logger.recordOutput("OrthogonalToTag/Using HF", false);
      }
    }

    globalTargetPose =
        RobotState.getInstance().getPose().plus(new Transform2d(new Pose2d(), targetState.pose));
    Logger.recordOutput("OrthogonalToTag/globalTargetPose", globalTargetPose);

    globalCurrentPose =
        RobotState.getInstance().getPose().plus(new Transform2d(new Pose2d(), currentRelativePose));
    Logger.recordOutput("OrthogonalToTag/globalCurrentPose", globalCurrentPose);

    /** Calculate speeds and set robot */
    // ChassisSpeeds chassisSpeeds =
    //     swerve.swerveFollower.calculateRobotRelativeSpeeds(currentRelativePose, targetState);

    /** Alternative method of sending robot to pose */
    double xPID = translationPID.calculate(currentRelativePose.getX(), targetState.pose.getX());
    double yPID = translationPID.calculate(currentRelativePose.getY(), targetState.pose.getY());
    double wantedRotationSpeeds =
        rotationPID.calculate(
            currentRelativePose.getRotation().getRadians(), targetState.heading.getRadians());
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xPID, yPID, wantedRotationSpeeds, currentRelativePose.getRotation());

    Logger.recordOutput("OrthogonalToTag/ChassisSpeeds", chassisSpeeds);

    swerve.driveRobotRelative(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("OrthogonalToTag/ExecutingCommand...", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double distanceMetersErr =
        RobotState.getInstance()
            .getPose()
            .getTranslation()
            .getDistance(globalTargetPose.getTranslation());
    double angleDegreesErr =
        Math.abs(
            RobotState.getInstance()
                .getPoseRotation2d()
                .minus(globalTargetPose.getRotation())
                .getDegrees());

    Logger.recordOutput("OrthogonalToTag/distanceMetersErr", distanceMetersErr);
    Logger.recordOutput("OrthogonalToTag/angleDegreesErr", angleDegreesErr);

    if (distanceMetersErr < .05 && angleDegreesErr < 5) exitCommand = true;

    return timer.hasElapsed(15);
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

  /** returns the Transformations of the robot to the AprilTag target (focusedTag only) */
  public List<Transform3d> getTransform3dList() {

    List<Transform3d> list =
        vision.getPipelineResults().stream()
            .filter(result -> result.getValue().hasTargets())
            .filter(result -> result.getValue().getBestTarget().fiducialId == focusedTag)
            .map(
                result ->
                    result
                        .getKey()
                        .getRobotToCameraTransform()
                        .plus(result.getValue().getBestTarget().getBestCameraToTarget()))
            .collect(Collectors.toList());

    vision.getPipelineResults().stream()
        .filter(result -> result.getValue().hasTargets())
        .filter(result -> result.getValue().getBestTarget().fiducialId == focusedTag)
        .forEach(
            transform -> {
              Logger.recordOutput(
                  "OrthogonalToTag/" + transform.getKey() + "/robotToCameraTransform",
                  transform.getKey().getRobotToCameraTransform());
              Logger.recordOutput(
                  "OrthogonalToTag/" + transform.getKey() + "/cameraToTarget",
                  transform.getValue().getBestTarget().getBestCameraToTarget());
            });

    return list;
  }
}

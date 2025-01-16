// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.helpers.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.maps.Constants;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision extends SubsystemBase implements VisionIO {

  public record Result(PhotonPipelineResult result, PhotonPoseEstimator poseEstimator) {}

  public static PhotonVision mInstance;

  public static PhotonVision getInstance() {
    if (mInstance == null) {
      mInstance = new PhotonVision();
    }
    return mInstance;
  }

  /* Create Camera */
  public PhotonCamera frontCamera;
  public PhotonCamera backCamera;

  public VisionSystemSim visionSim;

  /* Camera 1 PhotonPoseEstimator. */
  public PhotonPoseEstimator frontCamera_photonEstimator;
  /* Camera 2 PhotonPoseEstimator. */
  public PhotonPoseEstimator backCamera_photonEstimator;

  PhotonPoseEstimator[] photonEstimators;

  /** Creates a new Photonvision. */
  private PhotonVision() {

    // This will take a bit of tweaking to get right. I'm fairly certain that remotehost is defined
    // in the photonvision ui.
    PortForwarder.add(5800, "limelight2.local", 5800);
    PortForwarder.add(5801, "limelight3.local", 5801);

    frontCamera = new PhotonCamera(Constants.cameraOne.CAMERA_ONE_NAME);
    backCamera = new PhotonCamera(Constants.cameraTwo.CAMERA_TWO_NAME);

    switch (Constants.currentMode) {
      case REAL:
        frontCamera_photonEstimator =
            new PhotonPoseEstimator(
                Constants.AprilTags.aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.cameraOne.ROBOT_TO_CAM);
        backCamera_photonEstimator =
            new PhotonPoseEstimator(
                Constants.AprilTags.aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.cameraTwo.ROBOT_TO_CAM);

        frontCamera_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        backCamera_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimators =
            new PhotonPoseEstimator[] {frontCamera_photonEstimator, backCamera_photonEstimator};
        break;

      case SIM:
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(Constants.AprilTags.aprilTagFieldLayout);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(62.5));
        cameraProp.setFPS(40);

        PhotonCameraSim cameraSim = new PhotonCameraSim(frontCamera, cameraProp);

        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-35), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        visionSim.addCamera(cameraSim, robotToCamera);

        break;

      case REPLAY:
        break;
    }
  }

  /**
   * IMPORTANT METHOD! Main method for updating PhotonVision Data!
   *
   * @param VisionData - This is a container object that stores all the data surrounding Vision.
   *     More information in Vision.java
   * @param currentEstimate - This is where the robot thinks it is at this moment, before it updates
   *     itself through the SwerveDrivePoseEstimator.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {

    /* Only an array in case we use multiple cameras. */
    List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> results = new ArrayList<>();

    results.addAll(
        frontCamera.getAllUnreadResults().stream()
            .map(result -> Map.entry(frontCamera_photonEstimator, result))
            .collect(Collectors.toList()));

    results.addAll(
        backCamera.getAllUnreadResults().stream()
            .map(result -> Map.entry(backCamera_photonEstimator, result))
            .collect(Collectors.toList()));

    // Resetting the poseEstimates every period?
    inputs.poseEstimates = new Pose2d[0];

    inputs.timestamp = estimateAverageTimestamp(results);

    /** If you have a target, then update the poseEstimate ArrayList to equal that. */
    if (hasAnyTarget(results)) {
      // inputs.results = results;

      inputs.poseEstimates = getPoseEstimatesArray(results);
      inputs.hasEstimate = true;

    } else {
      inputs.timestamp = inputs.timestamp;
      inputs.hasEstimate = false;
    }
  }

  /**
   * Only needed if there are multiple cameras, but used in this situation nonetheless.
   *
   * <p>Takes PhotonPipelineResults and a PhotonPoseEstimator object and pumps out an ArrayList with
   * the estimated Poses it can find with any targets it might have.
   *
   * @param results - Raw results gotten from the camera, through the getLatestResult() method.
   * @param photonEstimator - An array of pose estimators that match their corresponding pipeline
   *     result.
   * @return An ArrayList with Pose2d objects.
   */
  public Pose2d[] getPoseEstimatesArray(
      List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> results) {

    List<Pose2d> estimates = new ArrayList<>();

    for (int i = 0; i < results.size(); i++) {

      Optional<EstimatedRobotPose> estimatedPose =
          results.get(i).getKey().update(results.get(i).getValue());
      if (estimatedPose.isPresent()) {
        estimates.add(estimatedPose.get().estimatedPose.toPose2d());
      }
      estimates.removeIf(pose -> pose == null);
    }

    return estimates.toArray(new Pose2d[0]);
  }

  // public List<PhotonTrackedTarget> getTargetsList(PhotonPoseEstimator photonEstimator) {
  //   return photonEstimator.update().get().targetsUsed;
  // }

  public double estimateAverageTimestamp(
      List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> results) {
    double latestTimestamp = 0;
    int count = 0;
    for (Map.Entry<PhotonPoseEstimator, PhotonPipelineResult> result : results) {
      latestTimestamp = result.getValue().getTimestampSeconds();
      count++;
    }
    return latestTimestamp / count;
  }

  // broken should be
  public double[] getTimestampArray(PhotonPipelineResult[] results) {
    double[] timestamps = new double[results.length];
    for (int i = 0; i < results.length; i++) {
      timestamps[i] = results[i].getTimestampSeconds();
    }
    return timestamps;
  }

  /** If any of the results have targets, then return true. */
  public boolean hasAnyTarget(List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> results) {
    for (Map.Entry<PhotonPoseEstimator, PhotonPipelineResult> result : results) {
      if (result.getValue().hasTargets()) {
        return true;
      }
    }
    return false;
  }

  // public double alignToSpeaker(boolean alliance){
  //   //    double currentRotationRadians=
  // SwerveDrive.getAdjustedYaw(SwerveDrive.getSwerveDrivePoseEstimator().getEstimatedPosition().getRotation().getRadians());
  //      double xTransform;
  //      double yTransform;
  //      double alignAngle;
  //      // assume that you are facing the april tag, new fix in the works
  //      if (alliance){
  //       xTransform = Constants.AprilTags.aprilTagPoses[4].getX() -
  // SwerveDriveSubsystem.getInstance().getSwerveDrivePoseEstimator().getEstimatedPosition().getX();
  //       yTransform = Constants.AprilTags.aprilTagPoses[4].getY() -
  // SwerveDrive.getSwerveDrivePoseEstimator().getEstimatedPosition().getY();
  //       alignAngle = Math.atan2(xTransform, yTransform)+180;
  //      }
  //      else {
  //       xTransform = Constants.AprilTags.aprilTagPoses[7].getX() -
  // SwerveDrive.getSwerveDrivePoseEstimator().getEstimatedPosition().getX();
  //       yTransform = Constants.AprilTags.aprilTagPoses[7].getY() -
  // SwerveDrive.getSwerveDrivePoseEstimator().getEstimatedPosition().getY();
  //       alignAngle = Math.atan2(xTransform, yTransform);
  //      }

  /**
   * These methods below are kinda useless unless you have multiple cameras. check this out:
   * https://github.com/FRC1257/2024-Robot/blob/master/src/main/java/frc/robot/subsystems/vision/VisionIOPhoton.java#L152
   */
  // private PhotonPoseEstimator[] getAprilTagEstimators(Pose2d currentEstimate) {

  //       camera1_photonPoseEstimator.setReferencePose(currentEstimate);

  //       return new PhotonPoseEstimator[] { camera1_photonPoseEstimator };
  //   }

  // private PhotonPipelineResult[] getAprilTagResults() {

  //       PhotonPipelineResult camera1_result = getLatestResult(camera1);

  //       // printStuff("cam1", cam1_result);

  //       return new PhotonPipelineResult[] { camera1_result };
  //   }

  @Override
  public void periodic() {

    Logger.recordOutput("VisionData/HasTarget?", RobotState.getInstance().visionInputs.hasEstimate);

    // This method will be called once per scheduler run
  }
}

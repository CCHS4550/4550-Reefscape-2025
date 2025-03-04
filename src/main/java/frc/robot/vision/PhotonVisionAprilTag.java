// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.util.maps.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionAprilTag extends SubsystemBase implements VisionIO {

  public List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> results = new ArrayList<>();
  public List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> condensedResults =
      new ArrayList<>();
  public List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> trustedResults =
      new ArrayList<>();

  /* Create Camera */
  public PhotonCamera limelight3;
  public PhotonCamera limelight2p;
  // public PhotonCamera limelight3_2;

  /* Camera 1 PhotonPoseEstimator. */
  public PhotonPoseEstimator limelight3_photonEstimator;
  /* Camera 2 PhotonPoseEstimator. */
  public PhotonPoseEstimator limelight2p_photonEstimator;

  public PhotonPoseEstimator limelight3_2_photonEstimator;

  /** Creates a new Photonvision. */
  public PhotonVisionAprilTag() {

    /** 10.45.50.11:5800 */
    limelight3 = new PhotonCamera(Constants.cameraOne.CAMERA_ONE_NAME);

    /** 10.45.50.12:5800 */
    limelight2p = new PhotonCamera(Constants.cameraTwo.CAMERA_TWO_NAME);

    /** 10.45.50.13:5800 */
    // limelight3_2 = new PhotonCamera(Constants.cameraThree.CAMERA_THREE_NAME);

    limelight3_photonEstimator =
        new PhotonPoseEstimator(
            Constants.AprilTags.APRIL_TAG_FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.cameraOne.ROBOT_TO_CAM);
    limelight2p_photonEstimator =
        new PhotonPoseEstimator(
            Constants.AprilTags.APRIL_TAG_FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.cameraTwo.ROBOT_TO_CAM);
    // limelight3_2_photonEstimator =
    //     new PhotonPoseEstimator(
    //         Constants.AprilTags.APRIL_TAG_FIELD_LAYOUT,
    //         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //         Constants.cameraThree.ROBOT_TO_CAM);

    limelight3_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    limelight2p_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // limelight3_2_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  /**
   * IMPORTANT METHOD! Main method for updating PhotonVision Inputs!
   *
   * @param inputs - This is a container object that stores all the data surrounding Vision. More
   *     information in Vision.java
   * @param currentEstimate - This is where the robot thinks it is at this moment, before it updates
   *     itself through the SwerveDrivePoseEstimator.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {

    results.clear();

    results.addAll(
        limelight3.getAllUnreadResults().stream()
            .map(result -> Map.entry(limelight3_photonEstimator, result))
            .collect(Collectors.toList()));

    results.addAll(
        limelight2p.getAllUnreadResults().stream()
            .map(result -> Map.entry(limelight2p_photonEstimator, result))
            .collect(Collectors.toList()));

    // results.addAll(
    //     limelight3_2.getAllUnreadResults().stream()
    //         .map(result -> Map.entry(limelight3_2_photonEstimator, result))
    //         .collect(Collectors.toList()));

    condensedResults = condensePipelineResults(results);
    trustedResults = getTrustedResults(results, 0.03, 1);

    inputs.hasTarget = hasAnyTarget(results);

    if (condensedResults.size() > 0)
      inputs.focusedId = condensedResults.get(0).getValue().getBestTarget().fiducialId;

    // Resetting the poseEstimates every period?
    inputs.poseEstimates = new Pose2d[0];

    /** If you have a target, then update the poseEstimate ArrayList to equal that. */
    if (hasAnyTarget(results)) {

      inputs.poseEstimates = getPoseEstimatesArray(trustedResults);
      inputs.timestampArray =
          trustedResults.stream()
              .mapToDouble(result -> result.getValue().getTimestampSeconds())
              .toArray();
      inputs.hasEstimate = true;

    } else {
      inputs.poseEstimates = new Pose2d[0];
      inputs.timestampArray = new double[0];
      inputs.hasEstimate = false;
    }
  }

  @Override
  public List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> getPipelineResults() {
    return results;
  }

  @Override
  public List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> getCondensedPipelineResults() {
    return condensedResults;
  }

  @Override
  public List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> getTrustedResults(
      List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> rawResults,
      double allowedMaxAmbiguity,
      double allowedMaxDistance) {

    List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> trustedResults = new ArrayList<>();
    for (int i = 0; i < rawResults.size(); i++) {

      if (rawResults.get(i).getValue().getMultiTagResult().isPresent()
          && rawResults.get(i).getValue().getMultiTagResult().get().estimatedPose.ambiguity >= 0
          && rawResults.get(i).getValue().getMultiTagResult().get().estimatedPose.ambiguity
              <= allowedMaxAmbiguity) {

        double maxDistance = 0.0;
        List<Short> fiducialIDsUsed =
            rawResults.get(i).getValue().getMultiTagResult().get().fiducialIDsUsed;
        for (Short id : fiducialIDsUsed) {
          double distanceFromTag =
              Constants.AprilTags.TAG_MAP
                  .get(id.intValue())
                  .getTranslation()
                  .getDistance(RobotState.getInstance().getPose().getTranslation());
          if (distanceFromTag > maxDistance) maxDistance = distanceFromTag;
        }

        if (maxDistance > allowedMaxDistance) trustedResults.add(rawResults.get(i));
      }
    }

    return trustedResults;
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
      List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> poseResults) {

    List<Pose2d> estimates = new ArrayList<>();

    for (Map.Entry<PhotonPoseEstimator, PhotonPipelineResult> poseResult : poseResults) {

      Optional<EstimatedRobotPose> estimatedPose =
          poseResult.getKey().update(poseResult.getValue());
      // if (estimatedPose.isPresent()
      //     && estimatedPose.get().strategy == PoseStrategy.LOWEST_AMBIGUITY) {
      estimates.add(estimatedPose.get().estimatedPose.toPose2d());
      // }
    }

    // estimates.removeIf(pose -> pose == null);

    return estimates.toArray(new Pose2d[0]);
  }

  public List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> condensePipelineResults(
      List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> uncondensedResults) {

    List<Integer> fudicialIDList = new ArrayList<>();
    for (Map.Entry<PhotonPoseEstimator, PhotonPipelineResult> condensedResult :
        uncondensedResults) {
      if (condensedResult.getValue().hasTargets())
        fudicialIDList.add(condensedResult.getValue().getBestTarget().fiducialId);
    }

    condensedResults.removeIf(result -> result.getValue().hasTargets() == false);

    condensedResults.removeIf(
        result ->
            result.getValue().getBestTarget().getFiducialId() != getPlurality(fudicialIDList));

    return condensedResults;
  }

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

  /** If any of the results have targets, then return true. */
  public boolean hasAnyTarget(List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> results) {
    for (Map.Entry<PhotonPoseEstimator, PhotonPipelineResult> result : results) {
      if (result.getValue().hasTargets()) {
        return true;
      }
    }
    return false;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}

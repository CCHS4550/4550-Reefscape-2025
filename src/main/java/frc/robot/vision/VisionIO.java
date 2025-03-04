// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */

/** Helper Class! Well actually, helper interface. */
public interface VisionIO {

  /** This is a container class to contain all the vision data. */
  @AutoLog
  class VisionIOInputs {
    public Pose2d[] poseEstimates;
    public double[] timestampArray;

    public int focusedId;

    public int[] visibleCamera1Targets;
    public int[] visibleCamera2Targets;
    // public int[] camera3Targets = new int[0];

    public boolean hasEstimate;
    public boolean hasTarget;
  }

  /** Default method, defined in photonvision */
  public default void updateInputs(VisionIOInputs inputs) {}

  public default List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> getPipelineResults() {
    return new ArrayList<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>>();
  }

  public default List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>>
      getCondensedPipelineResults() {
    return new ArrayList<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>>();
  }

  public default List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> getTrustedResults(
      List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> rawResults,
      double allowedMaxAmbiguity,
      double allowedMaxDistance) {
    return new ArrayList<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>>();
  }

  public default List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>>
      condensePipelineResults() {
    return new ArrayList<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>>();
  }

  default int getPlurality(List<Integer> list) {
    // Map to store the frequency of each element
    Map<Integer, Integer> frequencyMap = new HashMap<>();

    // Count occurrences of each element
    for (Integer item : list) {
      frequencyMap.put(item, frequencyMap.getOrDefault(item, 0) + 1);
    }

    // Find the element with the highest frequency
    Integer plurality = -1;
    int maxCount = 0;
    for (Map.Entry<Integer, Integer> entry : frequencyMap.entrySet()) {
      if (entry.getValue() > maxCount) {
        maxCount = entry.getValue();
        plurality = entry.getKey();
      }
    }

    return (int) plurality;
  }
}

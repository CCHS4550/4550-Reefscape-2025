// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.helpers.vision;

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
    public Pose2d[] poseEstimates = new Pose2d[0];
    public double timestamp = 0;
    public double[] timestampArray = new double[0];

    public int focusedId = 0;

    public int[] visibleCamera1Targets = new int[0];
    public int[] visibleCamera2Targets = new int[0];
    // public int[] camera3Targets = new int[0];

    public boolean hasEstimate = false;
  }

  /** Default method, defined in photonvision */
  public default void updateInputs(VisionIOInputs inputs) {}

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
    Integer plurality = null;
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.helpers.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.maps.Constants;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionSim extends SubsystemBase implements VisionIO {

  public static PhotonVisionSim mInstance;

  public static PhotonVisionSim getInstance() {
    if (mInstance == null) {
      mInstance = new PhotonVisionSim();
    }
    return mInstance;
  }

  /* Create Camera */
  public PhotonCamera leftCamera;
  public PhotonCamera rightCamera;

  public VisionSystemSim visionSim;

  PhotonCameraSim leftCameraSim;
  PhotonCameraSim rightCameraSim;

  /* Camera 1 PhotonPoseEstimator. */
  public PhotonPoseEstimator leftCamera_photonEstimator;
  /* Camera 2 PhotonPoseEstimator. */
  public PhotonPoseEstimator rightCamera_photonEstimator;

  PhotonPoseEstimator[] photonEstimators;

  public List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> results = new ArrayList<>();
  public List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> condensedResults =
      new ArrayList<>();

  /** Creates a new Photonvision. */
  private PhotonVisionSim() {

    leftCamera = new PhotonCamera(Constants.cameraOne.CAMERA_ONE_NAME);
    rightCamera = new PhotonCamera(Constants.cameraTwo.CAMERA_TWO_NAME);

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(Constants.AprilTags.aprilTagFieldLayout);

    SimCameraProperties leftCameraProp = new SimCameraProperties();
    SimCameraProperties rightCameraProp = new SimCameraProperties();
    leftCameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(62.5));
    leftCameraProp.setFPS(40);
    rightCameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(62.5));
    rightCameraProp.setFPS(40);

    leftCameraSim = new PhotonCameraSim(leftCamera, leftCameraProp);
    rightCameraSim = new PhotonCameraSim(rightCamera, rightCameraProp);

    // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
    // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
    Translation3d robotToLeftCameraTrans = new Translation3d(0.3302, 0.3048, 0.2032);
    Translation3d robotToRightCameraTrans = new Translation3d(0.3302, -0.3048, 0.2032);
    // and pitched 15 degrees up.
    Rotation3d robotToLeftCameraRot = new Rotation3d(0, Math.toRadians(-0), Math.toRadians(15));
    Rotation3d robotToRightCameraRot = new Rotation3d(0, Math.toRadians(-0), Math.toRadians(-15));

    Transform3d robotToLeftCamera = new Transform3d(robotToLeftCameraTrans, robotToLeftCameraRot);
    Transform3d robotToRightCamera =
        new Transform3d(robotToRightCameraTrans, robotToRightCameraRot);

    visionSim.addCamera(leftCameraSim, robotToLeftCamera);
    visionSim.addCamera(rightCameraSim, robotToRightCamera);

    // Enable the raw and processed streams. These are enabled by default.
    leftCameraSim.enableRawStream(true);
    rightCameraSim.enableRawStream(true);
    leftCameraSim.enableProcessedStream(true);
    rightCameraSim.enableProcessedStream(true);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    leftCameraSim.enableDrawWireframe(true);
    rightCameraSim.enableDrawWireframe(true);
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

    /* Only an array in case we use multiple cameras. */
    results.clear();

    results.addAll(
        leftCameraSim.getCamera().getAllUnreadResults().stream()
            .map(result -> Map.entry(leftCamera_photonEstimator, result))
            .collect(Collectors.toList()));

    results.addAll(
        rightCameraSim.getCamera().getAllUnreadResults().stream()
            .map(result -> Map.entry(rightCamera_photonEstimator, result))
            .collect(Collectors.toList()));

    condensedResults = results;
    condensedResults = condensePipelineResults();

    visionSim.update(RobotState.getInstance().getPose());

    inputs.timestampArray = new double[] {Timer.getFPGATimestamp()};

    // Resetting the poseEstimates every period?
    inputs.poseEstimates = new Pose2d[0];

    inputs.timestamp = Timer.getFPGATimestamp();

    /** If you have a target, then update the poseEstimate ArrayList to equal that. */
    if (visionSim.getVisionTargets().size() > 0) {

      inputs.poseEstimates = new Pose2d[] {visionSim.getRobotPose().toPose2d()};
      inputs.hasEstimate = true;

    } else {
      inputs.timestamp = inputs.timestamp;
      inputs.hasEstimate = false;
    }
  }

  @Override
  public List<Map.Entry<PhotonPoseEstimator, PhotonPipelineResult>> condensePipelineResults() {
    List<Integer> fudicialIDList = new ArrayList<>();
    for (Map.Entry<PhotonPoseEstimator, PhotonPipelineResult> condensedResult : condensedResults) {
      fudicialIDList.add(condensedResult.getValue().getBestTarget().fiducialId);
    }
    condensedResults.removeIf(
        result ->
            result.getValue().getBestTarget().getFiducialId() != getPlurality(fudicialIDList));

    return condensedResults;
  }

  public static int getPlurality(List<Integer> list) {
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

  @Override
  public void periodic() {

    Logger.recordOutput("VisionData/HasTarget?", RobotState.getInstance().visionInputs.hasEstimate);

    // This method will be called once per scheduler run
  }
}

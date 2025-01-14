// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;


/** Add your docs here. */

/** Helper Class! Well actually, helper interface. */
public interface VisionIO {



  /** This is a container class to contain all the vision data. */
  @AutoLog
  class VisionIOInputs {
    public Pose2d[] poseEstimates = new Pose2d[0];
    public double timestamp = 0;
    public double[] timestampArray = new double[0];

    public int[] camera1Targets = new int[0];
    public int[] camera2Targets = new int[0];
    // public int[] camera3Targets = new int[0];

    public boolean hasEstimate = false;
  }

  @AutoLog
  class Visiontest {
    public double help;

  }

  /** Default method, defined in photonvision */
  public default void updateInputs(VisionIOInputs inputs, Pose2d estimate) {}
}

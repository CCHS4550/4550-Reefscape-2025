// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swervedrive;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.helpers.CCMotorController;
import frc.maps.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This class provides an implementation of high-frequency odometry; essentially, recieving data
 * from our modules at a higher frequency than normal.
 */
public class RealOdometryThread {

  private final List<CCMotorController> sparkMaxes = new ArrayList<>();
  /**
   * These double suppliers supply the method by which to get the odometry data. Use registerInput()
   * to define that method.
   */
  private final List<DoubleSupplier> sparkMaxInput = new ArrayList<>();

  private final List<DoubleSupplier> genericInput = new ArrayList<>();

  /** These queues contain all the odometry data, ready to be processed. */
  private final List<Queue<Double>> sparkMaxContainer = new ArrayList<>();

  private final List<Queue<Double>> genericContainer = new ArrayList<>();
  private final List<Queue<Double>> timestampContainer = new ArrayList<>();

  private static RealOdometryThread instance = null;
  /**
   * This is what actually creates the thread, and runs the run() method at a predefined period
   * (defined in start())
   */
  private Notifier notifier = new Notifier(this::run);

  public static RealOdometryThread getInstance() {
    if (instance == null) {
      instance = new RealOdometryThread();
    }
    return instance;
  }

  private RealOdometryThread() {
    notifier.setName("OdometryThread");
  }

  public void start() {
    if (timestampContainer.size() > 0) {
      notifier.startPeriodic(1.0 / Constants.SwerveConstants.ODOMETRY_FREQUENCY);
    }
  }

  /** Registers a MotorController signal to be read from the thread. */
  public Queue<Double> registerInput(CCMotorController motor, DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    SwerveDriveSubsystem.odometryLock.lock();
    try {
      sparkMaxes.add(motor);
      sparkMaxInput.add(signal);
      sparkMaxContainer.add(queue);
    } finally {
      SwerveDriveSubsystem.odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerInput(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    SwerveDriveSubsystem.odometryLock.lock();
    try {
      genericInput.add(signal);
      genericContainer.add(queue);
    } finally {
      SwerveDriveSubsystem.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampContainer() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    SwerveDriveSubsystem.odometryLock.lock();
    try {
      timestampContainer.add(queue);
    } finally {
      SwerveDriveSubsystem.odometryLock.unlock();
    }
    return queue;
  }

  private void run() {
    // Save new data to queues
    SwerveDriveSubsystem.odometryLock.lock();
    try {
      // Get sample timestamp
      double timestamp = RobotController.getFPGATime() / 1e6;

      // Read Spark values, mark invalid in case of error
      double[] sparkValues = new double[sparkMaxInput.size()];
      boolean isValid = true;
      for (int i = 0; i < sparkMaxInput.size(); i++) {
        sparkValues[i] = sparkMaxInput.get(i).getAsDouble();
        if (sparkMaxes.get(i).getLastError() != REVLibError.kOk) {
          isValid = false;
        }
      }

      // If valid, add values to queues
      if (isValid) {
        for (int i = 0; i < sparkMaxInput.size(); i++) {
          sparkMaxContainer.get(i).offer(sparkValues[i]);
        }
        for (int i = 0; i < genericInput.size(); i++) {
          genericContainer.get(i).offer(genericInput.get(i).getAsDouble());
        }
        for (int i = 0; i < timestampContainer.size(); i++) {
          timestampContainer.get(i).offer(timestamp);
        }
      }
    } finally {
      SwerveDriveSubsystem.odometryLock.unlock();
    }
  }
}

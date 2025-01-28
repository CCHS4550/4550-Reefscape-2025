// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.maps.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Map;

/** Add your docs here. */

/** Largely inspired of off */
public class PathWrapper {

  // This is what is input to the PathWrapper
  public record AutoFile(String fileName, boolean isChoreoTraj) {}

  // This is what is created.
  // private record PathData(PathPlannerTrajectory trajectory, Command autoCommand) {}

  ArrayList<Command> followCommands = new ArrayList<>();

  PathPlannerTrajectory initialTraj;
  PathPlannerPath initialPath;
  Pose2d initialPose;

  /**
   * Wraps all the paths associated with an Autonomous routine inside a container object.
   *
   * @param autoRoutine The Auto Routine associated with these paths.
   * @param initialHeading The very first heading of the autonomous routine, probably 3.14.
   * @param files an AutoFiles array with all the auto files.
   */
  public PathWrapper(
      CustomAutoChooser.AutoRoutine autoRoutine, Rotation2d initialHeading, AutoFile... files) {

    initialPath =
        files[0].isChoreoTraj
            ? getPathPlannerPathfromChoreo(files[0].fileName)
            : getPathPlannerPathfromPath(files[0].fileName);

    initialTraj =
        files[0].isChoreoTraj
            ? getChoreoTrajectoryAutoRoutine(files[0].fileName, new ChassisSpeeds(), initialHeading)
            : getPathPlannerTrajectoryAutoRoutine(
                files[0].fileName, new ChassisSpeeds(), initialHeading);

    followCommands.add(followTrajectory(initialTraj));

    initialPose = initialTraj.getInitialPose();
    //  initialPath.getPreviewStartingHolonomicPose();

    for (int i = 1; i < files.length; i++) {

      if (files[i].isChoreoTraj) {
        followCommands.add(
            followChoreo(
                // File name
                files[i].fileName,
                // Empty Chassis Speeds
                new ChassisSpeeds(),
                // Heading at the beginning of the path (End of the previous path)
                getPathPlannerPathfromChoreo(files[i - 1].fileName).getGoalEndState().rotation()));
      } else {
        followCommands.add(
            followPathPlanner(
                // File name
                files[i].fileName,
                // Empty Chassis Speeds
                new ChassisSpeeds(),
                // Heading at the beginning of the path (End of the previous path)
                getPathPlannerPathfromPath(files[i - 1].fileName).getGoalEndState().rotation()));
      }
    }
  }

  public Command setInitialPose() {
    return new InstantCommand(() -> RobotState.getInstance().setOdometry(initialPose));
  }

  public Command getStartingCommand() {
    return followCommands.get(0);
  }

  public Command getEndingCommand() {
    return followCommands.get((followCommands.size() - 1));
  }

  public Command getFollowCommand(int index) {
    return followCommands.get(index);
  }

  public Command getAllCommands() {
    SequentialCommandGroup allCommands = new SequentialCommandGroup();
    for (Command command : followCommands) {
      allCommands.addCommands(command);
    }
    return allCommands;
  }

  /** Primary Follow Commands */
  public static Command followChoreo(String filename, ChassisSpeeds speeds, Rotation2d heading) {

    return new FollowPathCommand(getChoreoTrajectoryAutoRoutine(filename, speeds, heading));
  }

  public static Command followPathPlanner(
      String filename, ChassisSpeeds speeds, Rotation2d heading) {

    return new FollowPathCommand(getPathPlannerTrajectoryAutoRoutine(filename, speeds, heading));
  }

  /** Get the PathPlannerTrajectory from a particular PathPlannerPath */
  private static PathPlannerTrajectory getChoreoTrajectoryAutoRoutine(
      String filename, ChassisSpeeds speeds, Rotation2d initialHeading) {
    return getPathPlannerPathfromChoreo(filename)
        .generateTrajectory(speeds, initialHeading, Constants.SwerveConstants.ROBOT_CONFIG);
  }

  private static PathPlannerTrajectory getPathPlannerTrajectoryAutoRoutine(
      String filename, ChassisSpeeds speeds, Rotation2d initialHeading) {
    return getPathPlannerPathfromPath(filename)
        .generateTrajectory(speeds, initialHeading, Constants.SwerveConstants.ROBOT_CONFIG);
  }

  /** Get PathPlannerPath from files */
  public static PathPlannerPath getPathPlannerPathfromPath(String filename) {
    try {
      return flipIfNecessary(PathPlannerPath.fromPathFile(filename));
    } catch (IOException e) {
      e.printStackTrace();
      System.err.println("Error reading the path file" + filename);
    } catch (org.json.simple.parser.ParseException e) {
      e.printStackTrace();
      System.err.println("Error parsing the path file" + filename);
    } catch (NullPointerException e) {
      System.err.print("File does not exist " + filename);
    }
    return null;
  }

  public static PathPlannerPath getPathPlannerPathfromChoreo(String filename) {
    try {
      return flipIfNecessary(
          PathPlannerPath.fromChoreoTrajectory(
              handleChoreoIndex(filename).getKey(), handleChoreoIndex(filename).getValue()));
    } catch (IOException e) {
      e.printStackTrace();
      System.err.println("Error reading the path file" + filename);
    } catch (org.json.simple.parser.ParseException e) {
      e.printStackTrace();
      System.err.println("Error parsing the path file" + filename);
    } catch (NullPointerException e) {
      System.err.println("File does not exist " + filename);
    }
    return null;
  }

  public static PathPlannerPath flipIfNecessary(PathPlannerPath path) {
    return Constants.isBlue() ? path : path.flipPath();
  }

  public static Map.Entry<String, Integer> handleChoreoIndex(String filename) {
    // Split the string into two parts
    int lastDotIndex = filename.lastIndexOf(".");
    if (lastDotIndex == -1) {
      throw new IllegalArgumentException(
          "Input string must contain a period '.' separating the path and number.");
    }

    // Extract the parts
    String path = filename.substring(0, lastDotIndex); // "examplepath"
    int number = Integer.parseInt(filename.substring(lastDotIndex + 1)); // 1

    return Map.entry(path, number);
  }

  /** Helper Classes for general cases outside this class. */
  public static Command followChoreo(String filename) {
    return new FollowPathCommand(
        getPathPlannerPathfromChoreo(filename)
            .generateTrajectory(
                SwerveDriveSubsystem.getInstance().getRobotRelativeSpeeds(),
                RobotState.getInstance().getPoseRotation2d(),
                Constants.SwerveConstants.ROBOT_CONFIG));
  }

  public static Command followPathPlanner(String filename) {
    return new FollowPathCommand(
        getPathPlannerPathfromPath(filename)
            .generateTrajectory(
                SwerveDriveSubsystem.getInstance().getRobotRelativeSpeeds(),
                RobotState.getInstance().getPoseRotation2d(),
                Constants.SwerveConstants.ROBOT_CONFIG));
  }

  public static PathPlannerTrajectory getChoreoTrajectory(String filename) {
    return getPathPlannerPathfromChoreo(filename)
        .generateTrajectory(
            new ChassisSpeeds(0, 0, 0),
            getPathPlannerPathfromChoreo(filename).getStartingHolonomicPose().get().getRotation(),
            Constants.SwerveConstants.ROBOT_CONFIG);
  }

  public static PathPlannerTrajectory getPathPlannerTrajectory(String filename) {
    return getPathPlannerPathfromPath(filename)
        .generateTrajectory(
            new ChassisSpeeds(0, 0, 0),
            getPathPlannerPathfromPath(filename).getStartingHolonomicPose().get().getRotation(),
            Constants.SwerveConstants.ROBOT_CONFIG);
  }

  public static PathPlannerTrajectory convertPathToTrajectory(
      PathPlannerPath goToCoral, ChassisSpeeds chassisSpeeds, Rotation2d rotation2d) {
    PathPlannerTrajectory traj =
        goToCoral.generateTrajectory(
            chassisSpeeds, rotation2d, Constants.SwerveConstants.ROBOT_CONFIG);
    return traj;
  }

  private static Command followTrajectory(PathPlannerTrajectory traj) {
    return new FollowPathCommand(traj);
  }
}

package frc.robot.autonomous;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;

//IMPORTANT!!!! TODO - set the desired cage in code before comp/add a way to do it in shuffleboard

public class pathToCage {
  public static PathPlannerPath goToProccessor(int cageNum) {
    Pose2d[] blueSideCoors = {}; // fill with the coordinates for blue cages
    Pose2d[] redSideCoors = {}; // same as above but with red
    Pose2d targetPose;
    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
        targetPose = blueSideCoors[cageNum];
      } else {
        targetPose = redSideCoors[cageNum];
      }
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic
    // rotation.
    // just in case you wanted the bot to hit a location before going to the reef
    // would probably find additional waypoint coors just like the offset coors, just put them in a
    // big ass 2d array and take them ass needed
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(targetPose);
    // Create the path using the waypoints created above

    PathConstraints constraints =
        new PathConstraints(
            3.0,
            3.0,
            2 * Math.PI,
            4 * Math.PI); // The constraints for this path. ngl dont know what the numbers are be
    // ready to change them
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use
    // unlimited constraints, only limited by motor torque and nominal battery voltage

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                targetPose
                    .getRotation()) // Goal end state. You can set a holonomic rotation here. If
            // using a differential drivetrain, the rotation will have no
            // effect.
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;
    return path;
  }
}
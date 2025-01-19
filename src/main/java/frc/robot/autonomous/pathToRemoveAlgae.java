package frc.robot.autonomous;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import org.photonvision.PhotonUtils;
// so many of the imports are unused lmao
// could also use this for algae removal auto align

public class pathToRemoveAlgae {
  // just uses position on the field to calculate closest side, could also use
  // best april tag method with a check for it being the right ones but I think this is better b/c
  // it doesn't need to have reef in los
  // but also this has a big O notation of like 6 so idk it might be bad
  public static Pose2d closestSide(Pose2d pos) {
    Pose2d[]
        blueSideCoors = {}; // fill with the coordinates for the center of each wall on the blue
    // reef
    Pose2d[] redSideCoors = {}; // same as above but with red
    Pose2d closestCoor;
    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
      closestCoor = blueSideCoors[0];
      for (int i = 0; i < blueSideCoors.length; i++) {
        if (PhotonUtils.getDistanceToPose(pos, closestCoor)
            > PhotonUtils.getDistanceToPose(pos, blueSideCoors[i])) {
          closestCoor = blueSideCoors[i];
        }
      }
    } else {
      closestCoor = redSideCoors[0];
      for (int i = 0; i < redSideCoors.length; i++) {
        if (PhotonUtils.getDistanceToPose(pos, closestCoor)
            > PhotonUtils.getDistanceToPose(pos, redSideCoors[i])) {
          closestCoor = redSideCoors[i];
        }
      }
    }
    return closestCoor;
  }
  // there is probably a way to calculate this rather than use a stupid amount of coordinates but I
  // cannot be bothered to do that rn
  // also its just a bunch of if statements so big O is actually low
  public static PathPlannerPath goToRemoveCoral(Pose2d closestCoor) {
    Pose2d targetPose = closestCoor;
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

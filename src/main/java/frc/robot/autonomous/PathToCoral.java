package frc.robot.autonomous;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.maps.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
// so many of the imports are unused lmao
public class PathToCoral{
    // just uses position on the field to calculate closest side, could also use 
    // best april tag method with a check for it being the right ones but I think this is better b/c it doesn't need to have reef in los
    // but also this has a big O notation of like 12 so idk it might be bad
    public Pose2d closestSide(Pose2d pos, boolean alliance){
        Pose2d[] blueSideCoors = {}; // fill with the coordinates for the center of each wall on the blue reef
        Pose2d[] redSideCoors = {}; // same as above but with red
        Pose2d[] closestCoor;
        if(alliance){
            closestCoor = blueSideCoors[0];
            for(int i = 0; i < blueSideCoors.length; i++){
                if(PhotonUtils.getDistanceToPose(pos, closestCoor) > PhotonUtils.getDistanceToPose(pos, blueSideCoors[i])){
                    closestCoor = blueSideCoors[i];
                }
            }
        }
        else{
            closestCoor = redSideCoors[0];
            for(int i = 0; i < redSideCoors.length; i++){
                if(PhotonUtils.getDistanceToPose(pos, closestCoor) > PhotonUtils.getDistanceToPose(pos, redSideCoors[i])){
                    closestCoor = redSideCoors[i];
                }
            }
        }
        return closestCoor;
    }
    // literally the same thing as above, execpt we dont technically need the detailed Pose 2d info on the center so im just returning a number to make some other steps easier
    // be sure to label what direction the numbers go around in.
    public int closestSideNum(Pose2d pos, boolean alliance){
        Pose2d[] blueSideCoors = {}; // fill with the coordinates for the center of each wall on the blue reef
        Pose2d[] redSideCoors = {}; // same as above but with red
        Pose2d[] closestCoor;
        int hexNumber = 0;
        if(alliance){
            closestCoor = blueSideCoors[0]; // finds the shortest length of all the options on your alliance, definitely could be optimized but lowkey its not that bad rn, big O of 6 max
            for(int i = 0; i < blueSideCoors.length; i++){
                if(PhotonUtils.getDistanceToPose(pos, closestCoor) > PhotonUtils.getDistanceToPose(pos, blueSideCoors[i])){
                    closestCoor = blueSideCoors[i];
                    hexNumber = i;
                }
            }
        }
        else{
            closestCoor = redSideCoors[0];
            for(int i = 0; i < redSideCoors.length; i++){
                if(PhotonUtils.getDistanceToPose(pos, closestCoor) > PhotonUtils.getDistanceToPose(pos, redSideCoors[i])){
                    closestCoor = redSideCoors[i];
                    hexNumber = i;
                }
            }
        }
        return hexNumber
    }


    // there is probably a way to calculate this rather than use a stupid amount of coordinates but I cannot be bothered to do that rn
    // also its just a bunch of if statements so big O is actually low
    public PathPlannerPath goToCoral(int hexNumber, boolean alliance, int side){
        Pose2d[][] blueOffsets = {{}}; // 2d array, One axis represents which side it is according to hexnumber, the internal axis of 2 represents the coordinates of the offsets for each side. 6x2 array
        Pose2d[][] redOffsets = {{}}; // same thing but for red
        Pose2d targetPose;
        //just filters our goal pose into one pose2d
        if (alliance){
            targetPose = blueOffsets[hexNumber][side];
        }
        else{
            targetPose = redOffsets[hexNumber][side];
        }
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        //just in case you wanted the bot to hit a location before going to the reef
        //would probably find waypoint coors just like the offset coors, just put them in a big ass 2d array and take them ass needed
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        );
        // Create the path using the waypoints created above
        
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path. ngl dont know what the numbers are be ready to change them
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
        
        
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(targetPose) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }

}

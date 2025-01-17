package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.maps.Constants;

public class pathToTrajectory {
    // public static PathPlannerTrajectory covertPathToTrajectory(PathPlannerPath path, ChassisSpeeds startSpeeds, Rotation2d startRotation){
    //     PathPlannerTrajectory traj = path.generateTrajectory(startSpeeds, startRotation, Constants.SwerveConstants.ROBOT_CONFIG);
    //     return traj;
    // }

    public static PathPlannerTrajectory convertPathToTrajectory(PathPlannerPath goToCoral, ChassisSpeeds chassisSpeeds, Rotation2d rotation2d) {
        PathPlannerTrajectory traj = goToCoral.generateTrajectory(chassisSpeeds, rotation2d, Constants.SwerveConstants.ROBOT_CONFIG);
        return traj;
    }

    // public static PathPlannerTrajectory convertPathToTrajectory(PathPlannerPath goToCoral) {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'convertPathToTrajectory'");
    //}
}

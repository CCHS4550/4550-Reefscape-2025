public class pathToTrajectory {
    public PathPlannerTrajectory covertPathToTrajectory(PathPlannerTrajectory path, ChassisSpeeds startSpeeds, Rotation2d startRotation, RobotConfig Config){
        PathPlannerTrajectory traj = path.generateTrajectory(startSpeeds, startRotation, config);
        return traj;
    }
}

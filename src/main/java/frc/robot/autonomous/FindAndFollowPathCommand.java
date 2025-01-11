// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.autonomous;

// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
// import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
// import com.pathplanner.lib.pathfinding.LocalADStar;
// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.maps.Constants;
// import frc.robot.RobotState;
// import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
// import java.util.ArrayList;
// import java.util.List;
// import java.util.Optional;
// import org.littletonrobotics.junction.Logger;

// public class FindAndFollowPathCommand extends Command {

//   private LocalADStar pathFinder = new LocalADStar();

//   PathConstraints pathConstraints = new PathConstraints(2, 1, .5, .5);

//   Timer timer = new Timer();
//   Optional<PathPlannerPath> path;
//   Optional<PathPlannerTrajectory> trajectory;
//   PathPlannerTrajectoryState lastState, currentState, wantedState;

//   Pose2d targetPose = new Pose2d();

//   List<Pair<Translation2d, Translation2d>> obstacleList =
//       new ArrayList<Pair<Translation2d, Translation2d>>();

//   /** Creates a new FindAndFollowPathCommand. */
//   public FindAndFollowPathCommand(Pose2d targetPose) {
//     this.targetPose = targetPose;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//     timer.reset();
//     timer.start();

//     pathFinder.setStartPosition(RobotState.getInstance().getPose().getTranslation());
//     pathFinder.setGoalPosition(targetPose.getTranslation());

//     Pair<Translation2d, Translation2d> stage =
//         new Pair<>(new Translation2d(3, 6), new Translation2d(6.5, 2.5));

//     obstacleList.add(stage);

//     pathFinder.setDynamicObstacles(
//         obstacleList, RobotState.getInstance().getPose().getTranslation());

//     path =
//         Optional.ofNullable(
//             pathFinder.getCurrentPath(
//                 pathConstraints, new GoalEndState(0, targetPose.getRotation())));

//     if (path.isPresent()) {
//       trajectory =
//           Optional.ofNullable(
//               path.get()
//                   .generateTrajectory(
//                       Constants.SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
//                           SwerveDriveSubsystem.getInstance().desiredModuleStates),
//                       RobotState.getInstance().getPoseRotation2d()));
//     } else {
//       trajectory = Optional.empty();
//     }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     if (trajectory.isPresent()) {

//       /** The current time. */
//       double currentTime = timer.get();

//       /** Get the state of the robot at this current time in the path. */
//       wantedState = trajectory.get().sample(currentTime);

//       /** Add alliance transform! */

//       /** Create a ChassisSpeeds object to represent how the robot should be moving at this time.
// */
//       ChassisSpeeds chassisSpeeds =
//           SwerveDriveSubsystem.getInstance()
//               .swerveFollower
//               .calculateRobotRelativeSpeeds(RobotState.getInstance().currentPose, wantedState);

//       Logger.recordOutput(
//           "wantedAutoPose", wantedState.pose);

//       SwerveDriveSubsystem.getInstance().driveRobotRelative(chassisSpeeds);
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {

//     this.timer.stop();
//     SwerveDriveSubsystem.getInstance().stopModules();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {

//     if (path.isEmpty()) return true;

//     return timer.hasElapsed(trajectory.get().getTotalTimeSeconds());
//   }
// }

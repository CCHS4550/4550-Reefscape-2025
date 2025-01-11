// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.autonomous;

// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.RobotState;
// import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
// import java.util.ArrayList;

// /** Add your docs here. */

// /** Largely inspired of off */
// public class PathWrapper {

//   RobotConfig config = new RobotConfig(null, null, null, null)

//   // This is what is input to the PathWrapper
//   public record AutoFile(String fileName, boolean isChoreoTraj) {}

//   // This is what is created.
//   // private record PathData(PathPlannerTrajectory trajectory, Command autoCommand) {}

//   ArrayList<Command> followCommands = new ArrayList<>();

//   PathPlannerTrajectory initialTraj;
//   PathPlannerPath initialPath;
//   Pose2d initialPose;

//   /**
//    * Wraps all the paths associated with an Autonomous routine inside a container object.
//    *
//    * @param autoRoutine The Auto Routine associated with these paths.
//    * @param initialHeading The very first heading of the autonomous routine, probably 3.14.
//    * @param files an AutoFiles array with all the auto files.
//    */
//   public PathWrapper(
//       CustomAutoChooser.AutoRoutine autoRoutine, Rotation2d initialHeading, AutoFile... files) {

//     initialPath =
//         files[0].isChoreoTraj
//             ? PathPlannerPath.fromChoreoTrajectory(files[0].fileName)
//             : PathPlannerPath.fromPathFile(files[0].fileName);

//     initialTraj =
//         files[0].isChoreoTraj
//             ? getChoreoTrajectoryAutoRoutine(files[0].fileName, new ChassisSpeeds(),
// initialHeading)
//             : getPathPlannerTrajectoryAutoRoutine(
//                 files[0].fileName, new ChassisSpeeds(), initialHeading);

//     followCommands.add(followTrajectory(initialTraj));

//     initialPose = initialTraj.getInitialPose();
//     //  initialPath.getPreviewStartingHolonomicPose();

//     for (int i = 1; i < files.length; i++) {

//       if (files[i].isChoreoTraj) {
//         followCommands.add(
//             followChoreo(
//                 // File name
//                 files[i].fileName,
//                 // Empty Chassis Speeds
//                 new ChassisSpeeds(),
//                 // Heading at the beginning of the path (End of the previous path)
//                 PathPlannerPath.fromChoreoTrajectory(files[i - 1].fileName)
//                     .getGoalEndState()
//                     .rotation()));
//       } else {
//         followCommands.add(
//             followPathPlanner(
//                 // File name
//                 files[i].fileName,
//                 // Empty Chassis Speeds
//                 new ChassisSpeeds(),
//                 // Heading at the beginning of the path (End of the previous path)
//                 PathPlannerPath.fromPathFile(files[i - 1].fileName)
//                     .getGoalEndState()
//                     .rotation()));
//       }
//     }
//   }

//   public Command setInitialPose() {
//     return new InstantCommand(() -> RobotState.getInstance().setOdometry(initialPose));
//   }

//   public Command getStartingCommand() {
//     return followCommands.get(0);
//   }

//   public Command getEndingCommand() {
//     return followCommands.get((followCommands.size() - 1));
//   }

//   public Command getFollowCommand(int index) {
//     return followCommands.get(index);
//   }

//   public Command getAllCommands() {
//     SequentialCommandGroup allCommands = new SequentialCommandGroup();
//     for (Command command : followCommands) {
//       allCommands.addCommands(command);
//     }
//     return allCommands;
//   }

//   private Command followTrajectory(PathPlannerTrajectory traj) {
//     return new FollowPathCommand(traj);
//   }

//   public Command followChoreo(String filename, ChassisSpeeds speeds, Rotation2d heading) {

//     return new FollowPathCommand(getChoreoTrajectoryAutoRoutine(filename, speeds, heading));
//   }

//   public Command followPathPlanner(String filename, ChassisSpeeds speeds, Rotation2d heading) {

//     return new FollowPathCommand(getPathPlannerTrajectoryAutoRoutine(filename, speeds, heading));
//   }

//   private PathPlannerTrajectory getChoreoTrajectoryAutoRoutine(
//       String filename, ChassisSpeeds speeds, Rotation2d initialHeading) {

//     return PathPlannerPath.fromChoreoTrajectory(filename).generateTrajectory(speeds,
// initialHeading);
//   }

//   private PathPlannerTrajectory getPathPlannerTrajectoryAutoRoutine(
//       String filename, ChassisSpeeds speeds, Rotation2d initialHeading) {

//     return PathPlannerPath.fromPathFile(filename).generateTrajectory(speeds, initialHeading);
//   }

//   /** Helper Classes for general cases. */
//   public static Command followChoreo(String filename) {

//     return new FollowPathCommand(
//         PathPlannerPath.fromChoreoTrajectory(filename)
//             .generateTrajectory(
//                 SwerveDriveSubsystem.getInstance().getRobotRelativeSpeeds(),
//                 RobotState.getInstance().getPoseRotation2d()));
//   }

//   public static Command followPathPlanner(String filename) {
//     return new FollowPathCommand(
//         PathPlannerPath.fromPathFile(filename)
//             .generateTrajectory(
//               SwerveDriveSubsystem.getInstance().getRobotRelativeSpeeds(),
//                 RobotState.getInstance().getPoseRotation2d()));
//   }

//   public static PathPlannerTrajectory getChoreoTrajectory(String filename) {
//     return PathPlannerPath.fromChoreoTrajectory(filename)
//         .generateTrajectory(
//             new ChassisSpeeds(0, 0, 0),
//             PathPlannerPath.fromChoreoTrajectory(filename)
//                 .getStartingHolonomicPose()
//                 .get().getRotation());
//   }

//   public static PathPlannerTrajectory getPathPlannerTrajectory(String filename) {
//     return PathPlannerPath.fromPathFile(filename)
//         .generateTrajectory(
//             new ChassisSpeeds(0, 0, 0),
//             PathPlannerPath.fromPathFile(filename)
//             .getStartingHolonomicPose()
//             .get().getRotation());
//   }
// }
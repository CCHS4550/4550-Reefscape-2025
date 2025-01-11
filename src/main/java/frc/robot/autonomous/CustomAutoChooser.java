// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.autonomous;

// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// // import frc.robot.RobotContainer;
// import frc.robot.RobotState;

// /** Add your docs here. */
// public class CustomAutoChooser {

//   public static CustomAutoChooser mInstance;

//   public static CustomAutoChooser getInstance() {
//     if (mInstance == null) {
//       mInstance = new CustomAutoChooser();
//     }
//     return mInstance;
//   }

//   /** All the Auto Names. */
//   public enum AutoRoutine {
//     FOLLOW_ONE_METER,
//     FOLLOW_TWO_METER,
//     FOLLOW_ONE_METER_PP,
//     WORKSHOP_TEST,
//     EMPTY
//   }

//   // PathWrapper autoRoutine1 =
//   //     new PathWrapper(
//   //         AutoRoutine.EMPTY,
//   //         new AutoFile("path1", false),
//   //         new AutoFile("path2", false),
//   //         new AutoFile("path3", false),
//   //         new AutoFile("path4", false),
//   //         new AutoFile("path5", false));

//   /**
//    * This is what puts the options on Smart Dashboard, but instead of doing it by itself, we have
// to
//    * populate it manually.
//    */
//   private final SendableChooser<AutoRoutine> autoChooser = new SendableChooser<>();

//   public CustomAutoChooser() {
//     // this.trajectories = trajectories;

//     autoChooser.setDefaultOption("EMPTY", AutoRoutine.EMPTY);
//     autoChooser.addOption("Auto 1 Name", AutoRoutine.FOLLOW_ONE_METER);
//     autoChooser.addOption("Auto 2 Name", AutoRoutine.FOLLOW_TWO_METER);
//     autoChooser.addOption("Auto 3 Name", AutoRoutine.FOLLOW_ONE_METER_PP);
//     autoChooser.addOption("Auto 4 Name", AutoRoutine.WORKSHOP_TEST);
//     SmartDashboard.putData("Custom AutoChooser", autoChooser);

//     // autoChooser.addOption("Auto 3 Name", Autos.AUTO3);

//   }

//   /**
//    * Here is an example of how it should be formatted, give or take. This blob is copied exactly
//    * from 2910
//    */
//   //     private Command followAndDoChargingStationAndHome(RobotContainer container,
//   // PathPlannerTrajectory trajectory) {
//   //     SequentialCommandGroup homeArmCommand = new SequentialCommandGroup();
//   //     homeArmCommand.addCommands(new ArmToPoseCommand(container.getArmSubsystem(),
//   // ArmPoseConstants.STOW));
//   //     homeArmCommand.addCommands(new SimultaneousHomeArmCommand(container.getArmSubsystem()));

//   //     SequentialCommandGroup chargingStationCommand = new SequentialCommandGroup();
//   //     chargingStationCommand.addCommands(follow(container, trajectory));
//   //     chargingStationCommand.addCommands(new
//   // AutoBalanceOnChargeStationCommand(container.getDrivetrainSubsystem()));

//   //     return chargingStationCommand.alongWith(homeArmCommand);
//   // }

//   //   public Command auto1Command() {
//   //     return follow("test1").andThen(new InstantCommand());
//   //   }

//   public Command followOneMeter() {
//     SequentialCommandGroup c = new SequentialCommandGroup();
//     c.addCommands(followChoreoTestCommand("TYLERPATH", new Rotation2d()));
//     c.addCommands(
//         new FindAndFollowPathCommand(new Pose2d(new Translation2d(8, 4), new Rotation2d())));
//     return c;
//   }

//   public Command followTwoMeter() {
//     SequentialCommandGroup c = new SequentialCommandGroup();
//     c.addCommands(followChoreoTestCommand("SUHITPATH", new Rotation2d()));
//     // c.addCommands(
//     //     new FindAndFollowPathCommand(new Pose2d(new Translation2d(8, 4), new Rotation2d())));
//     return c;
//   }

//   public Command followOneMeterPP() {
//     return followPathPlannerTestCommand("1Meter", new Rotation2d());
//   }

//   public Command workshopTest() {
//     return followPathPlannerTestCommand("Workshop Test", new Rotation2d());
//   }

//   private Command intakeCommand() {
//     return new InstantCommand();
//   }

//   public Command autoRoutine1() {

//     SequentialCommandGroup c = new SequentialCommandGroup();
//     // Do not add file extensions!

//     c.addCommands(followChoreoTestCommand("traj1", new Rotation2d()));

//     c.addCommands(intakeCommand());

//     c.addCommands(followChoreoTestCommand("path2", new Rotation2d()));

//     return c;
//   }

//   public Command autoRoutine2() {
//     return new InstantCommand();
//   }

//   public Command autoRoutine3() {
//     return new InstantCommand();
//   }

//   public Command autoRoutine4() {
//     return new InstantCommand();
//   }

//   public Command getSelectedCustomCommand() {

//     switch (autoChooser.getSelected()) {
//         //   case AUTO1:
//         //     return auto1Command();
//       case FOLLOW_ONE_METER:
//         return followOneMeter();
//       case FOLLOW_TWO_METER:
//         return followTwoMeter();
//       case EMPTY:
//         return new InstantCommand();
//       case FOLLOW_ONE_METER_PP:
//         return followOneMeterPP();
//       case WORKSHOP_TEST:
//         return workshopTest();
//       default:
//         return new InstantCommand();
//     }
//   }

//   /**
//    * Creates a simpler follow helper method that simply requires the .traj file name from the
//    * deploy/choreo directory.
//    *
//    * @param pathname - The path name, no extension.
//    * @return - A follow Command!
//    */
//   public static Command followChoreoTestCommand(String pathname, Rotation2d initialHeading) {

//     PathPlannerTrajectory trajectory = PathWrapper.getChoreoTrajectory(pathname);
//     return new SequentialCommandGroup(
//         new InstantCommand(
//             () ->
//                 RobotState.getInstance()
//                     .setOdometry(
//                         PathPlannerPath.fromChoreoTrajectory(pathname)
//                             .getStartingHolonomicPose().get())),
//         new InstantCommand(() -> RobotState.getInstance().setRotation2d(initialHeading)),
//         // new Pose2d(
//         //     PathWrapper.getChoreoTrajectory(pathname)
//         //         .getInitialState()
//         //         .positionMeters,
//         //     PathWrapper.getChoreoTrajectory(pathname).getInitialState().heading))),
//         new FollowPathCommand(PathWrapper.getChoreoTrajectory(pathname)));
//   }

//   public static Command followPathPlannerTestCommand(String pathname, Rotation2d initialHeading)
// {
//     return new SequentialCommandGroup(
//         new InstantCommand(
//             () ->
//                 RobotState.getInstance()
//                     .setOdometry(
//                         PathPlannerPath.fromChoreoTrajectory(pathname)
//                         .getStartingHolonomicPose().get())),
//         new InstantCommand(() -> RobotState.getInstance().setRotation2d(initialHeading)),
//         new FollowPathCommand(PathWrapper.getPathPlannerTrajectory(pathname)));
//   }
// }

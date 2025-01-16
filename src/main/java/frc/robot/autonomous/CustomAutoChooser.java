// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;

/** Add your docs here. */
public class CustomAutoChooser {

  public static CustomAutoChooser mInstance;

  public static CustomAutoChooser getInstance() {
    if (mInstance == null) {
      mInstance = new CustomAutoChooser();
    }
    return mInstance;
  }

  /** All the Auto Names. */
  public enum AutoRoutine {
    AUTOROUTINE1,
    AUTOROUTINE2,
    AUTOROUTINE3,
    AUTOROUTINE4,
    EMPTY
  }

  PathWrapper pathWrapper1 =
      new PathWrapper(
          AutoRoutine.AUTOROUTINE1,
          Rotation2d.fromRadians(Math.PI),
          new PathWrapper.AutoFile("examplepath.1", false),
          new PathWrapper.AutoFile("examplepath.2", false),
          new PathWrapper.AutoFile("examplepath.3", false),
          new PathWrapper.AutoFile("examplepath.4", false),
          new PathWrapper.AutoFile("examplepath.5", false));

  /**
   * This is what puts the options on Smart Dashboard, but instead of doing it by itself, we have to
   * populate it manually.
   */
  private final SendableChooser<AutoRoutine> autoChooser = new SendableChooser<>();

  public CustomAutoChooser() {

    autoChooser.setDefaultOption("EMPTY", AutoRoutine.EMPTY);
    autoChooser.addOption("Auto 1 Name", AutoRoutine.AUTOROUTINE1);
    autoChooser.addOption("Auto 2 Name", AutoRoutine.AUTOROUTINE1);
    autoChooser.addOption("Auto 3 Name", AutoRoutine.AUTOROUTINE1);
    autoChooser.addOption("Auto 4 Name", AutoRoutine.AUTOROUTINE1);
    SmartDashboard.putData("Custom AutoChooser", autoChooser);

    // autoChooser.addOption("Auto 3 Name", Autos.AUTO3);

  }

  /**
   * Here is an example of how it should be formatted, give or take. This blob is copied exactly
   * from 2910
   */
  //     private Command followAndDoChargingStationAndHome(RobotContainer container,
  // PathPlannerTrajectory trajectory) {
  //     SequentialCommandGroup homeArmCommand = new SequentialCommandGroup();
  //     homeArmCommand.addCommands(new ArmToPoseCommand(container.getArmSubsystem(),
  // ArmPoseConstants.STOW));
  //     homeArmCommand.addCommands(new SimultaneousHomeArmCommand(container.getArmSubsystem()));

  //     SequentialCommandGroup chargingStationCommand = new SequentialCommandGroup();
  //     chargingStationCommand.addCommands(follow(container, trajectory));
  //     chargingStationCommand.addCommands(new
  // AutoBalanceOnChargeStationCommand(container.getDrivetrainSubsystem()));

  //     return chargingStationCommand.alongWith(homeArmCommand);
  // }

  public Command autoRoutine1() {

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper1.getStartingCommand());
    c.addCommands(pathWrapper1.getFollowCommand(1));
    c.addCommands(pathWrapper1.getFollowCommand(2));

    c.addCommands(followChoreoTestCommand("path2", new Rotation2d()));

    return c;
  }

  public Command autoRoutine2() {
    return new InstantCommand();
  }

  public Command autoRoutine3() {
    return new InstantCommand();
  }

  public Command autoRoutine4() {
    return new InstantCommand();
  }

  public Command getSelectedCustomCommand() {

    switch (autoChooser.getSelected()) {
        //   case AUTO1:
        //     return auto1Command();
      case AUTOROUTINE1:
        return autoRoutine1();
      case AUTOROUTINE2:
        return autoRoutine2();
      case AUTOROUTINE3:
        return autoRoutine3();
      case AUTOROUTINE4:
        return autoRoutine4();
      case EMPTY:
        return new InstantCommand();
      default:
        return new InstantCommand();
    }
  }

  /**
   * Creates a simpler follow helper method that simply requires the .traj file name from the
   * deploy/choreo directory.
   *
   * @param pathname - The path name, no extension.
   * @return - A follow Command!
   */
  public static Command followChoreoTestCommand(String pathname, Rotation2d initialHeading) {

    return new SequentialCommandGroup(
        new InstantCommand(
            () ->
                RobotState.getInstance()
                    .setOdometry(
                        PathWrapper.getPathPlannerPathfromChoreo(pathname)
                            .getStartingHolonomicPose()
                            .get())),
        new InstantCommand(() -> RobotState.getInstance().setRotation2d(initialHeading)),
        // new Pose2d(
        //     PathWrapper.getChoreoTrajectory(pathname)
        //         .getInitialState()
        //         .positionMeters,
        //     PathWrapper.getChoreoTrajectory(pathname).getInitialState().heading))),
        new FollowPathCommand(PathWrapper.getChoreoTrajectory(pathname)));
  }

  public static Command followPathPlannerTestCommand(String pathname, Rotation2d initialHeading) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () ->
                RobotState.getInstance()
                    .setOdometry(
                        PathWrapper.getPathPlannerPathfromPath(pathname)
                            .getStartingHolonomicPose()
                            .get())),
        new InstantCommand(() -> RobotState.getInstance().setRotation2d(initialHeading)),
        new FollowPathCommand(PathWrapper.getPathPlannerTrajectory(pathname)));
  }
}

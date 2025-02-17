// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.helpers.vision.VisionIO;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.Arrays;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Add your docs here. */
public class CustomAutoChooser {

  SwerveDriveSubsystem swerve;
  Superstructure superstructure;
  VisionIO vision;

  /** All the Auto Names. */
  public enum AutoRoutine {

    /** Left Cage 1, 4 piece */
    PITTSBURGH,
    /** Left Cage 2 Taxi */
    CENTENNIAL,
    /** Left Cage 3 Taxi */
    ENGLEWOOD,
    /** Left Wall, 1 piece and Collect */
    PORTLAND,
    /** Left Wall 2 piece */
    DENVER,
    /** Right Cage 1, 4 piece */
    DETROIT,
    /** Right Cage 1, 3 piece */
    SAN_DIEGO,
    /** Right Cage 1, 2 piece */
    HOUSTON,
    /** Right Cage 3, 1 piece */
    BUFFALO,
    /** Right Cage 3, 3 piece */
    SACRAMENTO,
    /** Right Cage 3, 2 piece */
    TRENTON,
    /** Right Wall, 1 piece */
    CHICAGO,
    /** Right Wall, 2 piece */
    AUSTIN,
    /** Middle, 1 piece */
    SEATTLE,
    /** Middle, 1 piece */
    CHARLOTTE,
    /** Left Cage 1, 3 piece */
    MANHATTAN,
    /** Left Wall Taxi */
    AURORA,
    /** Do nothing */
    EMPTY
  }

  /**
   * This is what puts the options on Smart Dashboard, but instead of doing it by itself, we have to
   * populate it manually.
   */
  private final LoggedDashboardChooser<AutoRoutine> autoChooser =
      new LoggedDashboardChooser<>("Autonomous Routine Chooser");

  public CustomAutoChooser(
      SwerveDriveSubsystem swerve, Superstructure superstructure, VisionIO vision) {
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.vision = vision;

    autoChooser.addDefaultOption("EMPTY", AutoRoutine.EMPTY);
    Arrays.stream(AutoRoutine.values())
        .forEach(auto -> autoChooser.addOption(auto.toString(), auto));
  }

  public Command runPittsburgh() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.PITTSBURGH,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Pittsburgh.0", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Pittsburgh.1", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Pittsburgh.2", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Pittsburgh.3", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Pittsburgh.4", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Pittsburgh.5", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Pittsburgh.6", true));

    SequentialCommandGroup c = new SequentialCommandGroup();

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(superstructure.setWantedSuperstateCommand(SuperState.CORAL_STATION_BACK));
    c.addCommands(pathWrapper.getFollowCommand(1));
    c.addCommands(pathWrapper.getFollowCommand(2));
    c.addCommands(pathWrapper.getFollowCommand(3));
    c.addCommands(pathWrapper.getFollowCommand(4));
    c.addCommands(pathWrapper.getFollowCommand(5));
    c.addCommands(pathWrapper.getFollowCommand(6));

    return c;
  }

  public Command runCentennial() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("LCg2 - Centennial.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));

    return c;
  }

  public Command runEnglewood() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("LCg3 - Englewood.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));

    return c;
  }

  public Command runPortland() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("LW - A4 - Portland.0", true),
            new PathWrapper.AutoFile("LW - A4 - Portland.1", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(pathWrapper.getFollowCommand(1));

    return c;
  }

  public Command runDenver() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("LW - A4B4 - Denver.0", true),
            new PathWrapper.AutoFile("LW - A4B4 - Denver.1", true),
            new PathWrapper.AutoFile("LW - A4B4 - Denver.2", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(pathWrapper.getFollowCommand(1));
    c.addCommands(pathWrapper.getFollowCommand(2));

    return c;
  }

  public Command runDetroit() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RCg1 - F2C4D4E4 - Detroit.0", true),
            new PathWrapper.AutoFile("RCg1 - F2C4D4E4 - Detroit.1", true),
            new PathWrapper.AutoFile("RCg1 - F2C4D4E4 - Detroit.2", true),
            new PathWrapper.AutoFile("RCg1 - F2C4D4E4 - Detroit.3", true),
            new PathWrapper.AutoFile("RCg1 - F2C4D4E4 - Detroit.4", true),
            new PathWrapper.AutoFile("RCg1 - F2C4D4E4 - Detroit.5", true),
            new PathWrapper.AutoFile("RCg1 - F2C4D4E4 - Detroit.6", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(pathWrapper.getFollowCommand(1));
    c.addCommands(pathWrapper.getFollowCommand(2));
    c.addCommands(pathWrapper.getFollowCommand(3));
    c.addCommands(pathWrapper.getFollowCommand(4));
    c.addCommands(pathWrapper.getFollowCommand(5));
    c.addCommands(pathWrapper.getFollowCommand(6));

    return c;
  }

  public Command runSanDiego() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RCg1 - F2D4C4 - San Diego.0", true),
            new PathWrapper.AutoFile("RCg1 - F2D4C4 - San Diego.1", true),
            new PathWrapper.AutoFile("RCg1 - F2D4C4 - San Diego.2", true),
            new PathWrapper.AutoFile("RCg1 - F2D4C4 - San Diego.3", true),
            new PathWrapper.AutoFile("RCg1 - F2D4C4 - San Diego.4", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(pathWrapper.getFollowCommand(1));
    c.addCommands(pathWrapper.getFollowCommand(2));
    c.addCommands(pathWrapper.getFollowCommand(3));
    c.addCommands(pathWrapper.getFollowCommand(4));

    return c;
  }

  public Command runHouston() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RCg2 - E4D4 - Houston.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(pathWrapper.getFollowCommand(1));
    c.addCommands(pathWrapper.getFollowCommand(2));
    c.addCommands(pathWrapper.getFollowCommand(3));
    c.addCommands(pathWrapper.getFollowCommand(4));
    c.addCommands(pathWrapper.getFollowCommand(5));

    return c;
  }

  public Command runBuffalo() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RCg3 - D4 - Buffalo.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));

    return c;
  }

  public Command runSacramento() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RCg3 - R - D4C4B4 - Sacramento.0", true),
            new PathWrapper.AutoFile("RCg3 - R - D4C4B4 - Sacramento.1", true),
            new PathWrapper.AutoFile("RCg3 - R - D4C4B4 - Sacramento.2", true),
            new PathWrapper.AutoFile("RCg3 - R - D4C4B4 - Sacramento.3", true),
            new PathWrapper.AutoFile("RCg3 - R - D4C4B4 - Sacramento.4", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(pathWrapper.getFollowCommand(1));
    c.addCommands(pathWrapper.getFollowCommand(2));
    c.addCommands(pathWrapper.getFollowCommand(3));
    c.addCommands(pathWrapper.getFollowCommand(4));

    return c;
  }

  public Command runTrenton() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RCg3 - R- D4C4 - Trenton.0", true),
            new PathWrapper.AutoFile("RCg3 - R- D4C4 - Trenton.1", true),
            new PathWrapper.AutoFile("RCg3 - R- D4C4 - Trenton.2", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(pathWrapper.getFollowCommand(1));
    c.addCommands(pathWrapper.getFollowCommand(2));

    return c;
  }

  public Command runChicago() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RW - B4 - Chicago.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));

    return c;
  }

  public Command runAustin() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RW - B4A4 - Austin.0", true),
            new PathWrapper.AutoFile("RW - B4A4 - Austin.1", true),
            new PathWrapper.AutoFile("RW - B4A4 - Austin.2", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(pathWrapper.getFollowCommand(1));
    c.addCommands(pathWrapper.getFollowCommand(2));

    return c;
  }

  public Command runSeattle() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("Z - G4 - Seattle.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));

    return c;
  }

  public Command runCharlotte() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("Z - H4 - Charlotte.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));

    return c;
  }

  public Command runManhattan() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("LCg1 - I2K4L4 - Manhattan.0", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4 - Manhattan.1", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4 - Manhattan.2", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4 - Manhattan.3", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4 - Manhattan.4", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    /* Score */
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(AlignCommands.frontAlignToReefLeft(swerve, vision));
    /* Pick up */
    c.addCommands(pathWrapper.getFollowCommand(1));
    c.addCommands(AlignCommands.frontAlignToCoralStationRight(swerve, vision));
    /* Score */
    c.addCommands(pathWrapper.getFollowCommand(2));
    c.addCommands(AlignCommands.backAlignToReefLeft(swerve, vision));
    /* Pick up */
    c.addCommands(pathWrapper.getFollowCommand(3));
    c.addCommands(AlignCommands.frontAlignToCoralStationRight(swerve, vision));
    /* Score */
    c.addCommands(pathWrapper.getFollowCommand(4));
    c.addCommands(AlignCommands.backAlignToReefRight(swerve, vision));

    return c;
  }

  public Command runAurora() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.CENTENNIAL,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("LW - Aurora.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));

    return c;
  }

  public Command getSelectedCustomCommand() {

    switch (autoChooser.get()) {
      case PITTSBURGH:
        return runPittsburgh();
      case CENTENNIAL:
        return runCentennial();
      case ENGLEWOOD:
        return runEnglewood();
      case PORTLAND:
        return runPortland();
      case DENVER:
        return runDenver();
      case DETROIT:
        return runDetroit();
      case SAN_DIEGO:
        return runSanDiego();
      case HOUSTON:
        return runHouston();
      case BUFFALO:
        return runBuffalo();
      case SACRAMENTO:
        return runSacramento();
      case TRENTON:
        return runTrenton();
      case CHICAGO:
        return runChicago();
      case AUSTIN:
        return runAustin();
      case SEATTLE:
        return runSeattle();
      case CHARLOTTE:
        return runCharlotte();
      case MANHATTAN:
        return runManhattan();
      case AURORA:
        return runAurora();
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
  public static Command followChoreoTestCommand(
      String pathname, Rotation2d initialHeading, SwerveDriveSubsystem swerve) {

    return new SequentialCommandGroup(
        new InstantCommand(
            () ->
                RobotState.getInstance()
                    .setOdometry(
                        PathWrapper.getPathPlannerPathfromChoreo(pathname)
                            .getStartingHolonomicPose()
                            .get()
                            .getTranslation())),
        new InstantCommand(() -> RobotState.getInstance().setRotation2d(initialHeading)),
        // new Pose2d(
        //     PathWrapper.getChoreoTrajectory(pathname)
        //         .getInitialState()
        //         .positionMeters,
        //     PathWrapper.getChoreoTrajectory(pathname).getInitialState().heading))),
        new FollowPathCommand(PathWrapper.getChoreoTrajectory(pathname), swerve));
  }

  public static Command followPathPlannerTestCommand(
      String pathname, Rotation2d initialHeading, SwerveDriveSubsystem swerve) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () ->
                RobotState.getInstance()
                    .setOdometry(
                        PathWrapper.getPathPlannerPathfromPath(pathname)
                            .getStartingHolonomicPose()
                            .get()
                            .getTranslation())),
        new InstantCommand(() -> RobotState.getInstance().setRotation2d(initialHeading)),
        new FollowPathCommand(PathWrapper.getPathPlannerTrajectory(pathname), swerve));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.superstructure.arm.ArmSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.wrist.WristSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.vision.VisionIO;
import java.util.Arrays;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This file compiles all the other autonomous files and sends them as one big
 * SequentialCommandGroup when autonomous is initialized. <br>
 * </br> Inspired by 2910 in 2023:
 * https://github.com/FRCTeam2910/2023CompetitionRobot-Public/blob/main/src/main/java/org/frcteam2910/c2023/util/PathChooser.java
 * <br>
 * </br> Take a look at this doc to understand the naming convention:
 * https://docs.google.com/document/d/e/2PACX-1vQw3eDluz8uo6XJKM-GdU-AAQzpGMUkd_6Zf4veUTayhUvbuNG5aprDZKgNVv8HWc22MCN-eD0zrgSl/pub
 */
public class CustomAutoChooser {

  private AlgaeSubsystem algae;
  private ArmSubsystem arm;
  private ClimberSubsystem climber;
  private ElevatorSubsystem elevator;
  private IntakeSubsystem intake;
  private SwerveDriveSubsystem swerve;
  private WristSubsystem wrist;

  private VisionIO vision;

  private Superstructure superstructure;

  /** All the Auto Names. */
  public enum AutoRoutine {

    /** Left Cage 1, 4 piece */
    SHANGHAI,
    /** Left Cage 2 Taxi */
    CENTENNIAL,
    /** Left Cage 3 Taxi */
    ENGLEWOOD,
    /** Left Wall, 1 piece and Collect */
    TOKYO,
    /** Left Wall 2 piece */
    DENVER,
    /** Right Cage 1, 4 piece */
    DETROIT,
    /** Right Cage 1, 3 piece */
    SAN_DIEGO,
    /** Right Cage 1, 2 piece */
    HOUSTON,
    /** Right Cage 3, 1 piece */
    BEIJING,
    /** Right Cage 3, 3 piece */
    SACRAMENTO,
    /** Right Cage 3, 2 piece */
    ATLANTA,
    /** Right Wall, 1 piece */
    BARCELONA,
    /** Right Wall, 2 piece */
    PARIS,
    /** Middle, 1 piece */
    SEATTLE,
    /** Middle, 1 piece */
    MUMBAI,
    /** Left Cage 1, 3 piece */
    MANHATTAN,
    /** Left Wall Taxi */
    AURORA,
    /** Left Cage 1, 1 piece */
    VLADIVOSTOK,
    /** Test */
    TEST,
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
      AlgaeSubsystem algae,
      ArmSubsystem arm,
      ClimberSubsystem climber,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake,
      SwerveDriveSubsystem swerve,
      WristSubsystem wrist,
      VisionIO vision,
      Superstructure superstructure) {
    this.algae = algae;
    this.arm = arm;
    this.climber = climber;
    this.elevator = elevator;
    this.intake = intake;
    this.swerve = swerve;
    this.wrist = wrist;

    this.vision = vision;

    this.superstructure = superstructure;

    autoChooser.addDefaultOption("EMPTY", AutoRoutine.EMPTY);
    Arrays.stream(AutoRoutine.values())
        .forEach(auto -> autoChooser.addOption(auto.toString(), auto));
  }

  public Command runShanghai() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.SHANGHAI,
            Rotation2d.fromRadians(0),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Shanghai.0", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Shanghai.1", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Shanghai.2", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Shanghai.3", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Shanghai.4", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Shanghai.5", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4J4 - Shanghai.6", true));

    SequentialCommandGroup c = new SequentialCommandGroup();

    c.addCommands(pathWrapper.setInitialPose());

    /* Score */
    // c.addCommands(
    //     // sequence(
    //     //     pathWrapper.getFollowCommand(0),
    //     //     // AlignCommands.backAlignToReefLeft(swerve, vision)
    //     //     //     .alongWith(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK))
    //     //     //     .withTimeout(5.5),
    //     //     intake.outtakeAuto())
    //     race(
    //         sequence(pathWrapper.getFollowCommand(0), waitSeconds(12)),
    //         sequence(
    //             waitSeconds(6),
    //             superstructure.setWantedSuperstateCommand(SuperState.L4_BACK),
    //             waitSeconds(2),
    //             intake.outtakeAuto(),
    //             waitSeconds(2))));

    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(0),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(sequence(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK)))
                .withTimeout(6),
            AlignCommands.backAlignToReefLeft(swerve, vision).withTimeout(6),
            waitSeconds(2),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                .withDeadline(pathWrapper.getFollowCommand(1))
                .withTimeout(5),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT))
                .withTimeout(1),
            intake.intakeAuto()));
    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(2),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(sequence(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK)))
                .withTimeout(6),
            AlignCommands.backAlignToReefLeft(swerve, vision).withTimeout(6),
            waitSeconds(2),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                .withDeadline(pathWrapper.getFollowCommand(3))
                .withTimeout(5),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT))
                .withTimeout(1),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(4),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(sequence(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK)))
                .withTimeout(6),
            AlignCommands.backAlignToReefLeft(swerve, vision).withTimeout(6),
            waitSeconds(2),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                .withDeadline(pathWrapper.getFollowCommand(5))
                .withTimeout(5),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT))
                .withTimeout(1),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(6),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(sequence(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK)))
                .withTimeout(6),
            AlignCommands.backAlignToReefLeft(swerve, vision).withTimeout(6),
            waitSeconds(2),
            intake.outtakeAuto()));

    return c;
  }

  public Command runVladivostok() {
    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.VLADIVOSTOK,
            Rotation2d.fromRadians(0),
            new PathWrapper.AutoFile("LCg1 - I4 - Vladivostok.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();

    c.addCommands(pathWrapper.setInitialPose());

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(0),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK))
                .withTimeout(5.5),
            intake.outtakeAuto()));

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
    /* Taxi! */
    c.addCommands(pathWrapper.getFollowCommand(0));

    return c;
  }

  public Command runEnglewood() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.ENGLEWOOD,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("LCg3 - Englewood.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    /* Taxi! */
    c.addCommands(pathWrapper.getFollowCommand(0));

    return c;
  }

  public Command runTokyo() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.TOKYO,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("LW - A4 - Tokyo.0", true),
            new PathWrapper.AutoFile("LW - A4 - Tokyo.1", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(0),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT)
                .alongWith(new WaitCommand(1).andThen(pathWrapper.getFollowCommand(1))),
            AlignCommands.frontAlignToCoralStationRight(swerve, vision)
                .alongWith(
                    superstructure
                        .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                        .withTimeout(3)),
            intake.intakeAuto()));

    return c;
  }

  public Command runDenver() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.DENVER,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("LW - A4B4 - Denver.0", true),
            new PathWrapper.AutoFile("LW - A4B4 - Denver.1", true),
            new PathWrapper.AutoFile("LW - A4B4 - Denver.2", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(0),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK))
                .withTimeout(5.5),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                .withDeadline(pathWrapper.getFollowCommand(1))
                .withTimeout(5),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT))
                .withTimeout(1),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(2).withTimeout(3),
            AlignCommands.backAlignToReefRight(swerve, vision)
                .alongWith(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK))
                .withTimeout(5.5),
            intake.outtakeAuto()),
        superstructure.setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT));

    return c;
  }

  public Command runDetroit() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.DETROIT,
            Rotation2d.fromRadians(0),
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

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(0),
            AlignCommands.backAlignToReefRight(swerve, vision)
                .alongWith(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK))
                .withTimeout(3),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                .withDeadline(pathWrapper.getFollowCommand(1))
                .withTimeout(5),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT))
                .withTimeout(1),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(2),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK))
                .withTimeout(4),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                .withDeadline(pathWrapper.getFollowCommand(3))
                .withTimeout(5),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT))
                .withTimeout(1),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(4),
            AlignCommands.backAlignToReefRight(swerve, vision)
                .alongWith(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK))
                .withTimeout(3),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                .withDeadline(pathWrapper.getFollowCommand(5))
                .withTimeout(5),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT))
                .withTimeout(1),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(6),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK))
                .withTimeout(3),
            intake.outtakeAuto()));

    return c;
  }

  public Command runSanDiego() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.SAN_DIEGO,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RCg1 - F2D4C4 - San Diego.0", true),
            new PathWrapper.AutoFile("RCg1 - F2D4C4 - San Diego.1", true),
            new PathWrapper.AutoFile("RCg1 - F2D4C4 - San Diego.2", true),
            new PathWrapper.AutoFile("RCg1 - F2D4C4 - San Diego.3", true),
            new PathWrapper.AutoFile("RCg1 - F2D4C4 - San Diego.4", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(0),
            AlignCommands.frontAlignToReefRight(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.L3_FRONT).withTimeout(3)),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT)
                .alongWith(new WaitCommand(1).andThen(pathWrapper.getFollowCommand(1))),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure
                        .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                        .withTimeout(3)),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(2),
            AlignCommands.backAlignToReefRight(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT)
                .alongWith(new WaitCommand(1).andThen(pathWrapper.getFollowCommand(3))),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure
                        .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                        .withTimeout(3)),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(4),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)),
            intake.outtakeAuto()));

    return c;
  }

  public Command runHouston() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.HOUSTON,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RCg2 - E4D4 - Houston.0", true),
            new PathWrapper.AutoFile("RCg2 - E4D4 - Houston.1", true),
            new PathWrapper.AutoFile("RCg2 - E4D4 - Houston.2", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(0),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK))
                .withTimeout(5.5),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                .withDeadline(pathWrapper.getFollowCommand(1))
                .withTimeout(5),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT))
                .withTimeout(1),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(2),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK))
                .withTimeout(5.5),
            intake.outtakeAuto()));

    return c;
  }

  public Command runBeijing() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.BEIJING,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RCg3 - D4 - Beijing.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    /* Score */
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(
        AlignCommands.backAlignToReefRight(swerve, vision)
            .alongWith(
                superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)));
    c.addCommands(intake.outtakeAuto());

    return c;
  }

  public Command runSacramento() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.SACRAMENTO,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RCg3 - R - D4C4B4 - Sacramento.0", true),
            new PathWrapper.AutoFile("RCg3 - R - D4C4B4 - Sacramento.1", true),
            new PathWrapper.AutoFile("RCg3 - R - D4C4B4 - Sacramento.2", true),
            new PathWrapper.AutoFile("RCg3 - R - D4C4B4 - Sacramento.3", true),
            new PathWrapper.AutoFile("RCg3 - R - D4C4B4 - Sacramento.4", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(0),
            AlignCommands.backAlignToReefRight(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT)
                .alongWith(new WaitCommand(1).andThen(pathWrapper.getFollowCommand(1))),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure
                        .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                        .withTimeout(3)),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(2),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT)
                .alongWith(new WaitCommand(1).andThen(pathWrapper.getFollowCommand(3))),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure
                        .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                        .withTimeout(3)),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(4),
            AlignCommands.backAlignToReefRight(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)),
            intake.outtakeAuto()));

    return c;
  }

  public Command runAtlanta() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.ATLANTA,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RCg3 - R- D4C4 - Atlanta.0", true),
            new PathWrapper.AutoFile("RCg3 - R- D4C4 - Atlanta.1", true),
            new PathWrapper.AutoFile("RCg3 - R- D4C4 - Atlanta.2", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(0),
            AlignCommands.backAlignToReefRight(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT)
                .alongWith(new WaitCommand(1).andThen(pathWrapper.getFollowCommand(1))),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure
                        .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                        .withTimeout(3)),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(2),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)),
            intake.outtakeAuto()));

    return c;
  }

  public Command runBarcelona() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.BARCELONA,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RW - B4 - Barcelona.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(
        AlignCommands.backAlignToReefLeft(swerve, vision)
            .alongWith(
                superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)));
    c.addCommands(intake.outtakeAuto());

    return c;
  }

  public Command runParis() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.PARIS,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("RW - B4A4 - Paris.0", true),
            new PathWrapper.AutoFile("RW - B4A4 - Paris.1", true),
            new PathWrapper.AutoFile("RW - B4A4 - Paris.2", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(0),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)),
            intake.outtakeAuto()));

    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT)
                .alongWith(new WaitCommand(1).andThen(pathWrapper.getFollowCommand(1))),
            AlignCommands.frontAlignToCoralStationRight(swerve, vision)
                .alongWith(
                    superstructure
                        .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                        .withTimeout(3)),
            intake.intakeAuto()));

    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(2),
            AlignCommands.backAlignToReefRight(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)),
            intake.outtakeAuto()));

    return c;
  }

  // seatlte works
  public Command runSeattle() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.SEATTLE,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("Z - G4 - Seattle.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(
        sequence(

            // pathWrapper.getFollowCommand(0),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK))
                .withTimeout(5.5),
            intake.outtakeAuto(),
            new WaitCommand(4),
            superstructure.setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT)));

    return c;
  }

  public Command runMumbai() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.MUMBAI,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("Z - H4 - Mumbai.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));
    c.addCommands(
        AlignCommands.backAlignToReefRight(swerve, vision)
            .alongWith(
                superstructure.setWantedSuperstateCommand(SuperState.L4_BACK).withTimeout(3)));
    c.addCommands(intake.outtakeAuto());
    c.addCommands(
        superstructure.setWantedSuperstateCommand(SuperState.WITHIN_FRAME_PERIMETER_DEFAULT));

    return c;
  }

  public Command runManhattan() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.MANHATTAN,
            Rotation2d.fromRadians(0),
            new PathWrapper.AutoFile("LCg1 - I2K4L4 - Manhattan.0", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4 - Manhattan.1", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4 - Manhattan.2", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4 - Manhattan.3", true),
            new PathWrapper.AutoFile("LCg1 - I2K4L4 - Manhattan.4", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!
    c.addCommands(pathWrapper.setInitialPose());
    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(0),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(sequence(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK)))
                .withTimeout(6),
            AlignCommands.backAlignToReefLeft(swerve, vision).withTimeout(6),
            waitSeconds(2),
            intake.outtakeAuto()));
    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                .withDeadline(pathWrapper.getFollowCommand(1))
                .withTimeout(5),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT))
                .withTimeout(1),
            intake.intakeAuto()));
    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(2),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(sequence(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK)))
                .withTimeout(6),
            AlignCommands.backAlignToReefLeft(swerve, vision).withTimeout(6),
            waitSeconds(2),
            intake.outtakeAuto()));
    /* Pick up */
    c.addCommands(
        sequence(
            superstructure
                .setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT)
                .withDeadline(pathWrapper.getFollowCommand(3))
                .withTimeout(5),
            AlignCommands.frontAlignToCoralStationLeft(swerve, vision)
                .alongWith(
                    superstructure.setWantedSuperstateCommand(SuperState.CORAL_STATION_FRONT))
                .withTimeout(1),
            intake.intakeAuto()));
    /* Score */
    c.addCommands(
        sequence(
            pathWrapper.getFollowCommand(4),
            AlignCommands.backAlignToReefLeft(swerve, vision)
                .alongWith(sequence(superstructure.setWantedSuperstateCommand(SuperState.L4_BACK)))
                .withTimeout(6),
            AlignCommands.backAlignToReefLeft(swerve, vision).withTimeout(6),
            waitSeconds(2),
            intake.outtakeAuto()));

    return c;
  }

  public Command runAurora() {

    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.AURORA,
            Rotation2d.fromRadians(Math.PI),
            new PathWrapper.AutoFile("LW - Aurora.0", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));

    return c;
  }

  public Command runTest() {
    PathWrapper pathWrapper =
        new PathWrapper(
            swerve,
            AutoRoutine.TEST,
            Rotation2d.fromRadians(0),
            new PathWrapper.AutoFile("TEST.0", true),
            new PathWrapper.AutoFile("TEST.1", true));

    SequentialCommandGroup c = new SequentialCommandGroup();
    // Do not add file extensions!

    c.addCommands(pathWrapper.setInitialPose());
    c.addCommands(pathWrapper.getFollowCommand(0));

    return c;
  }

  public Command getSelectedCustomCommand() {

    switch (autoChooser.get()) {
      case SHANGHAI:
        return runShanghai();
      case CENTENNIAL:
        return runCentennial();
      case ENGLEWOOD:
        return runEnglewood();
      case TOKYO:
        return runTokyo();
      case DENVER:
        return runDenver();
      case DETROIT:
        return runDetroit();
      case SAN_DIEGO:
        return runSanDiego();
      case HOUSTON:
        return runHouston();
      case BEIJING:
        return runBeijing();
      case SACRAMENTO:
        return runSacramento();
      case ATLANTA:
        return runAtlanta();
      case BARCELONA:
        return runBarcelona();
      case VLADIVOSTOK:
        return runVladivostok();
      case PARIS:
        return runParis();
      case SEATTLE:
        return runSeattle();
      case MUMBAI:
        return runMumbai();
      case MANHATTAN:
        return runManhattan();
      case AURORA:
        return runAurora();
      case TEST:
        return runTest();
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

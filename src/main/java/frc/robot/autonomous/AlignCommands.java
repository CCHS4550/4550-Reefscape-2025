// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.maps.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

/** Add your docs here. */
public class AlignCommands {

  public static Command frontAlignToReefLeft() {

    return new OrthogonalToTag(
        Constants.FieldPositionConstants.FRONT_REEF_LEFT_OFFSET,
        Constants.AprilTags.REEF_POSES,
        false);
  }

  public static Command frontAlignToReefRight() {
    Command alignCommand =
        new OrthogonalToTag(
            Constants.FieldPositionConstants.FRONT_REEF_RIGHT_OFFSET,
            Constants.AprilTags.REEF_POSES,
            false);
    return alignCommand;
  }

  public static Command backAlignToReefLeft() {
    Command alignCommand =
        new OrthogonalToTag(
            Constants.FieldPositionConstants.BACK_REEF_LEFT_OFFSET,
            Constants.AprilTags.REEF_POSES,
            false);
    return aboutFace().andThen(alignCommand);
  }

  public static Command backAlignToReefRight() {
    Command alignCommand =
        new OrthogonalToTag(
            Constants.FieldPositionConstants.BACK_REEF_RIGHT_OFFSET,
            Constants.AprilTags.REEF_POSES,
            false);
    return aboutFace().andThen(alignCommand);
  }

  public static Command backAlignToCoralStationLeft() {
    Command alignCommand =
        new OrthogonalToTag(
            Constants.FieldPositionConstants.CORAL_STATION_LEFT_OFFSET,
            Constants.AprilTags.CORAL_STATION_POSES,
            false);
    return alignCommand;
  }

  public static Command backAlignToCoralStationRight() {
    Command alignCommand =
        new OrthogonalToTag(
            Constants.FieldPositionConstants.CORAL_STATION_RIGHT_OFFSET,
            Constants.AprilTags.CORAL_STATION_POSES,
            false);
    return alignCommand;
  }

  public static Command AlignToProcessor() {
    Command alignCommand =
        new OrthogonalToTag(
            Constants.FieldPositionConstants.PROCESSOR_OFFSET,
            Constants.AprilTags.PROCESSOR_POSES,
            false);
    return alignCommand;
  }

  public static Command aboutFace() {
    double targetAngle = RobotState.getInstance().getPoseAngleRadians() + Math.PI;
    return new FunctionalCommand(
        () -> {
          SwerveDriveSubsystem.getInstance().rotationPID.reset();
        },
        () -> {
          double rotation =
              SwerveDriveSubsystem.getInstance()
                  .rotationPID
                  .calculate(RobotState.getInstance().getPoseAngleRadians(), targetAngle);
          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  0, 0, rotation, RobotState.getInstance().getPoseRotation2d());
          SwerveDriveSubsystem.getInstance().driveRobotRelative(chassisSpeeds);
        },
        (bool) -> SwerveDriveSubsystem.getInstance().stopModules(),
        () ->
            (Math.abs(targetAngle) - Math.abs(RobotState.getInstance().getPoseAngleRadians()))
                < 0.05,
        SwerveDriveSubsystem.getInstance());
  }
}

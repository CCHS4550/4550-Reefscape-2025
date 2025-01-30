// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.maps.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

/** Add your docs here. */
public class AlignCommands {

  public static Command frontAlignToReefLeft(SwerveDriveSubsystem swerve) {

    return new OrthogonalToTag(
        Constants.FieldPositionConstants.FRONT_REEF_LEFT_OFFSET,
        Constants.AprilTags.REEF_POSES,
        false,
        swerve);
  }

  public static Command frontAlignToReefRight(SwerveDriveSubsystem swerve) {
    Command alignCommand =
        new OrthogonalToTag(
            Constants.FieldPositionConstants.FRONT_REEF_RIGHT_OFFSET,
            Constants.AprilTags.REEF_POSES,
            false,
            swerve);
    return alignCommand;
  }

  public static Command backAlignToReefLeft(SwerveDriveSubsystem swerve) {
    Command alignCommand =
        new OrthogonalToTag(
            Constants.FieldPositionConstants.BACK_REEF_LEFT_OFFSET,
            Constants.AprilTags.REEF_POSES,
            false,
            swerve);
    return aboutFace(swerve).andThen(alignCommand);
  }

  public static Command backAlignToReefRight(SwerveDriveSubsystem swerve) {
    Command alignCommand =
        new OrthogonalToTag(
            Constants.FieldPositionConstants.BACK_REEF_RIGHT_OFFSET,
            Constants.AprilTags.REEF_POSES,
            false,
            swerve);
    return aboutFace(swerve).andThen(alignCommand);
  }

  public static Command backAlignToCoralStationLeft(SwerveDriveSubsystem swerve) {
    Command alignCommand =
        new OrthogonalToTag(
            Constants.FieldPositionConstants.CORAL_STATION_LEFT_OFFSET,
            Constants.AprilTags.CORAL_STATION_POSES,
            false,
            swerve);
    return alignCommand;
  }

  public static Command backAlignToCoralStationRight(SwerveDriveSubsystem swerve) {
    Command alignCommand =
        new OrthogonalToTag(
            Constants.FieldPositionConstants.CORAL_STATION_RIGHT_OFFSET,
            Constants.AprilTags.CORAL_STATION_POSES,
            false,
            swerve);
    return alignCommand;
  }

  public static Command AlignToProcessor(SwerveDriveSubsystem swerve) {
    Command alignCommand =
        new OrthogonalToTag(
            Constants.FieldPositionConstants.PROCESSOR_OFFSET,
            Constants.AprilTags.PROCESSOR_POSES,
            false,
            swerve);
    return alignCommand;
  }

  public static Command aboutFace(SwerveDriveSubsystem swerve) {
    double targetAngle = RobotState.getInstance().getPoseAngleRadians() + Math.PI;
    return new FunctionalCommand(
        () -> {
          swerve.rotationPID.reset();
        },
        () -> {
          double rotation =
              swerve.rotationPID.calculate(
                  RobotState.getInstance().getPoseAngleRadians(), targetAngle);
          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  0, 0, rotation, RobotState.getInstance().getPoseRotation2d());
          swerve.driveRobotRelative(chassisSpeeds);
        },
        (bool) -> swerve.stopModules(),
        () ->
            (Math.abs(targetAngle) - Math.abs(RobotState.getInstance().getPoseAngleRadians()))
                < 0.05,
        swerve);
  }

  public static Trigger hasTarget() {
    return new Trigger(() -> RobotState.getInstance().visionInputs.hasTarget)
        .and(() -> RobotState.getInstance().visionInputs.hasEstimate);
  }
}

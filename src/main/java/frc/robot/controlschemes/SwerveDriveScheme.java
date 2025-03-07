package frc.robot.controlschemes;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.autonomous.AlignCommands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.superstructure.arm.ArmSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.wrist.WristSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.vision.VisionIO;
import frc.util.maps.Constants;

/**
 * Control scheme for swerve drive. Includes movement, the toggle between field centric and robot
 * centric, and a button to zero the gyro.
 */
public class SwerveDriveScheme {

  // private static CommandXboxController controller;
  private static boolean fieldCentric = true;
  private static boolean orientationLocked = false;
  private static double orientationLockAngle;
  private static BooleanSupplier fieldCentricSupplier =
      () -> {
        return fieldCentric;
      };

  private static DoubleSupplier driveSpeedModifier = () -> .2;

  private static double turnSpeedModifier = 0.4;

  private static PIDController orientationLockPID;

  static {
    orientationLockPID = new PIDController(.5, 0, 0);
    orientationLockPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Configures the basic driving as well as buttons.
   *
   * @param algae Algae Subsystem
   * @param arm Arm Subsystem
   * @param climber Climber Subsystem
   * @param elevator Elevator Subsystem
   * @param intake Intake Subsystem
   * @param swerve Swerve Drive Subsystem
   * @param wrist Wrist Subsystem
   * @param superstructure Superstructure
   * @param controller Controller to bind controls to.
   */
  public static void configure(
      AlgaeSubsystem algae,
      ArmSubsystem arm,
      ClimberSubsystem climber,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake,
      SwerveDriveSubsystem swerve,
      WristSubsystem wrist,
      Superstructure superstructure,
      VisionIO vision,
      CommandXboxController controller) {

    Shuffleboard.getTab("Diagnostics")
        .getLayout("Swerve", "List")
        .add("isCentric", fieldCentric)
        .withWidget(BuiltInWidgets.kBooleanBox);
    Shuffleboard.getTab("Diagnostics")
        .addBoolean("Field Centric", fieldCentricSupplier)
        .withWidget(BuiltInWidgets.kToggleSwitch);

    swerve.resetTurnEncoders();

    SlewRateLimiter xRateLimiter =
        new SlewRateLimiter(
            Constants.SwerveConstants.DRIVE_RATE_LIMIT * 2,
            -Constants.SwerveConstants.DRIVE_RATE_LIMIT * 2,
            0);
    SlewRateLimiter yRateLimiter =
        new SlewRateLimiter(
            Constants.SwerveConstants.DRIVE_RATE_LIMIT * 2,
            -Constants.SwerveConstants.DRIVE_RATE_LIMIT * 2,
            0);

    // Set to slow mode for recreation
    // setSlowMode();
    setFastMode();

    // Sends this command into the command scheduler on repeat! Very important!
    swerve.setDefaultCommand(
        new RunCommand(
                () -> {
                  Logger.recordOutput("driveSpeedModifier", driveSpeedModifier);

                  // Set x, y, and turn speed based on joystick inputs
                  double xSpeed =
                      MathUtil.applyDeadband(-controller.getLeftY(), 0.3)
                          * Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND
                          * driveSpeedModifier.getAsDouble();

                  double ySpeed =
                      MathUtil.applyDeadband(-controller.getLeftX(), 0.3)
                          // MathUtil.applyDeadband(0, 0.01)
                          * Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND
                          * driveSpeedModifier.getAsDouble();

                  double turnSpeed = 0;
                  // || Math.abs(controller.getRightX()) > 0.15

                  if (!orientationLocked) {
                    orientationLockAngle = RobotState.getInstance().getPoseAngleRadians();
                    turnSpeed = MathUtil.applyDeadband(controller.getRightX(), 0.05);

                  } else {
                    turnSpeed =
                        orientationLockPID.calculate(
                            RobotState.getInstance().getPoseAngleRadians(), orientationLockAngle);
                  }

                  turnSpeed *= 2.0 * Math.PI * turnSpeedModifier;

                  xSpeed = xRateLimiter.calculate(xSpeed);
                  ySpeed = yRateLimiter.calculate(ySpeed);

                  // Constructs desired chassis speeds
                  ChassisSpeeds chassisSpeeds;

                  if (fieldCentric) {
                    // Relative to field
                    chassisSpeeds =
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeed,
                            ySpeed,
                            -turnSpeed,
                            Constants.isBlue
                                ? RobotState.getInstance().getPoseRotation2d()
                                : RobotState.getInstance()
                                    .getPoseRotation2d()
                                    .plus(Rotation2d.fromRadians(Math.PI)));
                    Logger.recordOutput("xSpeed", xSpeed);
                    Logger.recordOutput("ySpeed", ySpeed);

                  } else {
                    // Relative to robot
                    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, -turnSpeed);
                  }
                  swerve.driveRobotRelative(chassisSpeeds);
                },
                swerve)
            .withName("Swerve Controller Command"));

    configureButtons(
        algae, arm, climber, elevator, intake, swerve, wrist, superstructure, vision, controller);
  }

  /**
   * Configures buttons and their respective commands.
   *
   * @param algae
   * @param arm
   * @param climber
   * @param elevator
   * @param intake
   * @param swerve
   * @param wrist
   * @param superstructure
   * @param controller
   */
  private static void configureButtons(
      AlgaeSubsystem algae,
      ArmSubsystem arm,
      ClimberSubsystem climber,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake,
      SwerveDriveSubsystem swerve,
      WristSubsystem wrist,
      Superstructure superstructure,
      VisionIO vision,
      CommandXboxController controller) {

        final Trigger reefLeftTrigger = controller.leftTrigger().and(controller.x());
        final Trigger reefRightTrigger = controller.leftTrigger().and(controller.b());

        final Trigger reefBackLeftTrigger = controller.leftTrigger().and(controller.x()).and(controller.y());
        final Trigger reefBackRightTrigger = controller.leftTrigger().and(controller.b()).and(controller.y());

        // final Trigger reefLeftTrigger =
        // controller.leftTrigger().and(controller.x()).and(superstructure.isNotL4());
        // final Trigger reefRightTrigger =
        //  controller.leftTrigger().and(controller.b()).and(superstructure.isNotL4());

        // final Trigger reefBackLeftTrigger =
        // controller.leftTrigger().and(controller.x()).and(superstructure.isL4());
        // final Trigger reefBackRightTrigger =
        // controller.leftTrigger().and(controller.b()).and(superstructure.isL4());

        // final Trigger processorTrigger = controller.leftTrigger().and(controller.y());

    final Trigger coralStationLeftTrigger = controller.rightTrigger().and(controller.x());
    final Trigger coralStationRightTrigger = controller.rightTrigger().and(controller.b());

    // controller.y().onTrue(runOnce(() -> toggleFieldCentric()));

    reefLeftTrigger.whileTrue(AlignCommands.frontAlignToReefLeft(swerve, vision));
    reefRightTrigger.whileTrue(AlignCommands.frontAlignToReefRight(swerve, vision));

    reefBackLeftTrigger.whileTrue(AlignCommands.backAlignToReefLeft(swerve, vision));
    reefBackRightTrigger.whileTrue(AlignCommands.backAlignToReefRight(swerve, vision));

    // processorTrigger.whileTrue(AlignCommands.AlignToProcessor(swerve, vision));

    // controller.a().onTrue(runOnce(() -> RobotState.getInstance().resetRotation()));

    coralStationLeftTrigger.whileTrue(AlignCommands.frontAlignToCoralStationLeft(swerve, vision));
    coralStationRightTrigger.whileTrue(AlignCommands.frontAlignToCoralStationRight(swerve, vision));

    controller.leftTrigger().and(controller.y()).whileTrue(AlignCommands.alignToKnockAlgae(swerve, vision));

    controller
        .leftBumper()
        .onTrue(runOnce(() -> setFastMode()))
        .onFalse(runOnce(() -> setNormalMode()));
    controller
        .rightBumper()
        .onTrue(runOnce(() -> setSlowMode()))
        .onFalse(runOnce(() -> setNormalMode()));

      if(RobotController.getMatchTime() == 0){
        controller.setRumble(0, 1.0).withTimeout(0.5);
      }
  }
  /** Toggle field centric and robot centric driving. */
  private static void toggleFieldCentric() {
    fieldCentric = !fieldCentric;
  }

  private static void toggleOrientationLock(SwerveDriveSubsystem swerveDrive) {

    orientationLocked = !orientationLocked;
    if (orientationLocked) {
      orientationLockAngle = RobotState.getInstance().getPoseAngleRadians();
    }
  }

  private static void setFastMode() {
    driveSpeedModifier = () -> .7;
  }

  private static void setNormalMode() {
    driveSpeedModifier = () -> 0.4;
  }

  private static void setSlowMode() {
    driveSpeedModifier = () -> 0.05;
  }
}

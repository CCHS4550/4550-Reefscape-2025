package frc.robot.controlschemes;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.helpers.ControlScheme;
import frc.maps.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Control scheme for swerve drive. Includes movement, the toggle between field centric and robot
 * centric, and a button to zero the gyro.
 */
public class SwerveDriveScheme implements ControlScheme {

  // private static CommandXboxController controller;
  private static boolean fieldCentric = true;
  private static boolean orientationLocked = false;
  private static double orientationLockAngle;
  private static BooleanSupplier fieldCentricSupplier =
      () -> {
        return fieldCentric;
      };

  private static DoubleSupplier driveSpeedModifier = () -> 1.00;

  private static double turnSpeedModifier = 0.5;

  /**
   * Configures the basic driving as well as buttons.
   *
   * @param swerve The SwerveDrive object being configured.
   * @param controller The driving controller, initialized in RobotContainer.java.
   */
  public static void configure(
      SwerveDriveSubsystem swerve,
      //  Shooter shooter,
      //   Indexer indexer,
      //    int port
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
    // SlewRateLimiter turnRateLimiter =
    //     new SlewRateLimiter(Constants.SwerveConstants.TURN_RATE_LIMIT / 1.5);

    PIDController orientationLockPID = new PIDController(.5, 0, 0);

    orientationLockPID.enableContinuousInput(-Math.PI, Math.PI);
    // controller = new CommandXboxController(port);

    // Set to slow mode for recreation
    // setSlowMode();
    setFastMode();

    // Sends this command into the command scheduler on repeat! Very important!
    swerve.setDefaultCommand(
        new RunCommand(
                () -> {

                  // Set x, y, and turn speed based on joystick inputs
                  double xSpeed =
                      MathUtil.applyDeadband(-controller.getLeftY(), 0.01)
                          * Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND
                          * driveSpeedModifier.getAsDouble();

                  double ySpeed =
                      MathUtil.applyDeadband(-controller.getLeftX(), 0.01)
                          // MathUtil.applyDeadband(0, 0.01)
                          * Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND
                          * driveSpeedModifier.getAsDouble();

                  double turnSpeed = 0;
                  // || Math.abs(controller.getRightX()) > 0.15

                  if (!orientationLocked) {
                    orientationLockAngle = RobotState.getInstance().getPoseAngleRadians();
                    turnSpeed = MathUtil.applyDeadband(controller.getRightY(), 0.05);

                  } else {
                    turnSpeed =
                        orientationLockPID.calculate(
                            RobotState.getInstance().getPoseAngleRadians(), orientationLockAngle);
                  }

                  turnSpeed *= 2.0 * Math.PI * turnSpeedModifier;

                  // Limits acceleration and speed
                  // Possibly change the speed limiting to somewhere else (maybe a normalize
                  // function)
                  xSpeed = xRateLimiter.calculate(xSpeed);
                  ySpeed = yRateLimiter.calculate(ySpeed);
                  // turnSpeed = turnRateLimiter.calculate(turnSpeed);

                  // SmartDashboard.putNumber("xSpeed", xSpeed);
                  // SmartDashboard.putNumber("ySpeed", ySpeed);
                  // SmartDashboard.putNumber("turnSpeed", turnSpeed);

                  // Constructs desired chassis speeds
                  ChassisSpeeds chassisSpeeds;

                  if (fieldCentric) {
                    // Relative to field
                    chassisSpeeds =
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeed,
                            ySpeed,
                            -turnSpeed,
                            RobotState.getInstance().getPoseRotation2d());
                    Logger.recordOutput("xSpeed", xSpeed);
                    Logger.recordOutput("ySpeed", ySpeed);

                  } else {
                    // Relative to robot
                    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, -turnSpeed);
                  }
                  SwerveDriveSubsystem.getInstance().driveRobotRelative(chassisSpeeds);
                },
                swerve)
            .withName("Swerve Controller Command"));

    // configureButtons(swerve, controller);
  }

  /**
   * Configures buttons and their respective commands.
   *
   * @param swerveDrive The SwerveDrive object being configured.
   * @param port The controller port of the driving controller.
   */
  private static void configureButtons(
      SwerveDriveSubsystem swerve, CommandXboxController controller) {

    Trigger reefLeftTrigger = controller.leftTrigger().and(controller.x());
    Trigger reefRightTrigger = controller.leftTrigger().and(controller.b());

    Trigger reefBackLeftTrigger = controller.leftTrigger().and(controller.x()).and(controller.y());
    Trigger reefBackRightTrigger = controller.leftTrigger().and(controller.b()).and(controller.y());

    Trigger processorTrigger = controller.leftTrigger().and(controller.y());

    Trigger coralStationLeftTrigger = controller.rightTrigger().and(controller.x());
    Trigger coralStationRightTrigger = controller.rightTrigger().and(controller.b());

    controller.y().onTrue(runOnce(() -> toggleFieldCentric()));

    // controller.a().onTrue(runOnce(() -> setFastMode())).onFalse(runOnce(() -> setSlowMode()));
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
    driveSpeedModifier = () -> 1;
  }

  private static void setNormalMode() {
    driveSpeedModifier = () -> 0.75;
  }

  private static void setSlowMode() {
    driveSpeedModifier = () -> 0.3;
  }
}

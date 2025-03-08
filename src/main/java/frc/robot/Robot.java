// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.CustomAutoChooser;
import frc.util.BlinkinLEDController;
import frc.util.BlinkinLEDController.BlinkinPattern;
import frc.util.HighFrequencyThread;
import frc.util.maps.Constants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private RobotState robotState;

  @SuppressWarnings("unused")
  private RobotContainer robotContainer;

  private CustomAutoChooser autoChooser;

  private boolean browningOut = false;

  public Robot() {}

  @Override
  public void robotInit() {

    Constants.setCurrentMode();
    System.out.println(Constants.currentMode);
    Constants.checkAlliance();

    SmartDashboard.putBoolean("Browning Out?", browningOut);

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter("/U/logs")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher());
        Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // PortForwarder.add(5800, "10.45.50.11.5800", 5800);
    // PortForwarder.add(5801, "10.45.50.12.5800", 5801);

    // Unofficial REV-Compatible Logger
    // Used by SysID to log REV devices
    // This is the reason why it is largely unecessary to log Sparkmaxes, since it is done by this.
    Logger.registerURCL(URCL.startExternal());
    // Start AdvantageKit logger
    Logger.start();

    robotState = RobotState.getInstance();
    robotContainer = new RobotContainer();
    autoChooser = robotState.autoChooserInit();

    RobotState.getInstance().poseInit();
    // RobotState.getInstance().swerveModuleEncodersInit();
    // RobotState.getInstance().dashboardInit();

    RobotState.getInstance().resetPIDControllers();

    BlinkinLEDController.getInstance().setIfNotAlready(BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {

    Logger.recordOutput("isBlue", Constants.isBlue);

    switch (Constants.currentMode) {
      case REAL:
        if (RobotController.getBatteryVoltage() < 10) {
          browningOut = true;
        } else {
          browningOut = false;
        }

        break;

      case SIM:
        DriverStation.silenceJoystickConnectionWarning(true);

        break;

      case REPLAY:
        break;
    }

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    Constants.checkAlliance();
    HighFrequencyThread.getInstance().start();

    BlinkinLEDController.getInstance().setIfNotAlready(BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

    // RobotState.getInstance().resetPIDControllers();
    if (!RobotState.getInstance().poseInitialized) RobotState.getInstance().poseInit();
    autoChooser.getSelectedCustomCommand().schedule();

    System.out.println("Autonomous Routine Scheduled!");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // RobotState.getInstance().swerveModuleEncodersPeriodic();
    RobotState.getInstance().updateSwerveModulePositionsPeriodic();

    RobotState.getInstance().updateOdometryPose();
    RobotState.getInstance().updateVisionPose();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    Constants.checkAlliance();

    HighFrequencyThread.getInstance().start();

    BlinkinLEDController.getInstance().setPattern(BlinkinPattern.WHITE);

    if (!RobotState.getInstance().poseInitialized) RobotState.getInstance().poseInit();

    CameraServer.startAutomaticCapture();

    // RobotState.getInstance().resetPIDControllers();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // RobotState.getInstance().swerveModuleEncodersPeriodic();
    RobotState.getInstance().updateSwerveModulePositionsPeriodic();

    RobotState.getInstance().updateOdometryPose();
    RobotState.getInstance().updateVisionPose();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // RobotState.getInstance().resetPIDControllers();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // RobotState.getInstance().resetPIDControllers();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

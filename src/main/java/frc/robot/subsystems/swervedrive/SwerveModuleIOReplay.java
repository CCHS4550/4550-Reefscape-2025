// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.helpers.motorcontroller.CCMotorController;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class SwerveModuleIOReplay implements SwerveModuleIO {

  public SwerveModuleIOReplay(
      CCMotorController driveMotor,
      CCMotorController turnMotor,
      int absoluteEncoderChannel,
      double absoluteEncoderOffset,
      String name) {
    System.err.println(name + " SWERVEMODULE IN REPLAY");
  }

  public double getDrivePosition() {
    return 0;
  }

  public double getTurnPosition() {
    return 0;
  }

  public double getDriveSpeed() {
    return 0;
  }

  public double getTurnSpeed() {
    return 0;
  }

  public double getDriveVoltage() {
    return 0;
  }

  public double getTurnVoltage() {
    return 0;
  }

  public double getAbsoluteEncoderRadiansOffset() {
    return 0;
  }

  public double getAbsoluteEncoderRadiansNoOffset() {
    return 0;
  }

  public void resetEncoders() {}

  public void resetTurnEncoder() {}

  public SwerveModuleState getState() {
    return new SwerveModuleState();
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {}

  public void setDriveVelocity(double velocity) {}

  public void setTurnPosition(DoubleSupplier angle) {}

  public void stop() {}

  public void setDriveVoltage(double voltage) {}

  public void setTurnVoltage(double voltage) {}

  public void driveAndTurn(double driveSpeed, double turnSpeed) {}

  public String getName() {
    return "";
  }

  public void setName(String name) {}

  public double getTurnEncoderDistance() {
    return 0;
  }

  public double getTurnEncoderVelocity() {
    return 0;
  }

  public double getDriveEncoderDistance() {
    return 0;
  }

  public double getDriveEncoderVelocity() {
    return 0;
  }
}

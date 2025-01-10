package frc.robot.subsystems.swervedrive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.helpers.CCMotorController;

public interface SwerveModuleIO {

  @AutoLog
  class SwerveModuleData {

  }

  default double getDrivePosition() {
    return 0;
  }
  default double getTurnPosition() {
    return 0;
  }
  default double getDriveSpeed() {
    return 0;
  }
  default double getTurnSpeed() {
    return 0;
  }
  default double getDriveVoltage() {
    return 0;
  }
  default double getTurnVoltage() {
    return 0;
  }
  default double getAbsoluteEncoderRadiansOffset() {
    return 0;
  }
  default double getAbsoluteEncoderRadiansNoOffset() {
    return 0;
  }
  default void resetEncoders() {}
  default void resetTurnEncoder() {}
  
  default SwerveModuleState getState() {
    return new SwerveModuleState();
  }

  default void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {}

  default void setDriveVelocity(double velocity) {}

  default void setTurnPosition(DoubleSupplier angle) {}

  default void stop() {}

  default void setDriveVoltage(double voltage) {}
  default void setTurnVoltage(double voltage) {}

  default void driveAndTurn(double driveSpeed, double turnSpeed) {}

  default String getName() {
    return "";
  }
  default void setName(String name) {}

  default double getTurnEncoderDistance() {
    return 0;
  }
  default double getTurnEncoderVelocity() {
    return 0;
  }
  default double getDriveEncoderDistance() {
    return 0;
  }
  default double getDriveEncoderVelocity() {
    return 0;
  }

  @FunctionalInterface
  interface ModuleFactory {
    SwerveModuleIO create(
      CCMotorController driveMotor,
      CCMotorController turnMotor,
      int absoluteEncoderChannel,
      double absoluteEncoderOffset,
      String name);
  }
  
}



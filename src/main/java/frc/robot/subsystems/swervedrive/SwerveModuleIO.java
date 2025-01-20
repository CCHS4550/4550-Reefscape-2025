package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.helpers.CCMotorController;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

  @AutoLog
  class SwerveModuleInputs {

    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    // public boolean turnConnected = false;
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    // This is the final product of the RealOdometryThread.
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsMeters = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  default void updateInputs(SwerveModuleInputs inputs) {}

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
  public interface ModuleFactory {
    SwerveModuleIO create(
        CCMotorController driveMotor,
        CCMotorController turnMotor,
        int absoluteEncoderChannel,
        double absoluteEncoderOffset,
        String name);
  }
}

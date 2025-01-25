package frc.robot.subsystems.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.helpers.CCMotorController;
import frc.robot.subsystems.wrist.WristSubsystem.WristState;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

  @AutoLog
  public static class WristIOInputs {
    public double currentAngleDegrees = 0.0;
    public double currentAngleRadians = 0.0;

    public double pidOutput = 0.0;
    public double ffOutput = 0.0;
    public double appliedVoltage = 0.0;

    public double setpointAngleDegrees = 0.0;
    public double setpointAngleRadians = 0.0;
    public double setpointVelocity = 0.0;

    public double goalAngleRadians = 0.0;
    public double goalAngleDegrees = 0.0;
    public double goalVelocity = 0.0;
  }

  /** Update the data coming into the robot. */
  public default void updateInputs(WristIOInputs inputs) {}

  public default void holdAtState(WristState goalState) {}

  public default Command goToGoalState(State goalState, WristSubsystem wrist) {
    return new InstantCommand();
  }

  public default void setVoltage(Voltage voltage) {}

  public default double getVoltage() {
    return 0.0;
  }

  public default double getPIDFFOutput(State goalState) {
    return 0.0;
  }

  public default void stop() {}

  public default BooleanSupplier atSetpoint() {
    return () -> false;
  }

  /**
   * Gets the reading of the absolute encoder with offset.
   *
   * @return The value of the absolute encoder in radians with the offset applied.
   */
  default double getAbsoluteEncoderRadiansOffset() {
    return 0.0;
  }

  /**
   * Gets the reading of the absolute encoder with offset. Used for getting the offset.
   *
   * @return The value of the absolute encoder in radians without the offset applied.
   */
  default double getAbsoluteEncoderRadiansNoOffset() {
    return 0.0;
  }

  @FunctionalInterface
  interface IOFactory {
    WristIO create(CCMotorController motor);
  }
}

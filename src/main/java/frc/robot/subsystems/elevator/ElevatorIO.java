package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.helpers.CCMotorController;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  class ElevatorIOInputs {
    public double currentPositionRotations = 0.0;

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
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void holdAtState(ElevatorState goalState) {}

  public default Command goToGoalState(State goalState, ElevatorSubsystem elevator) {
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
   * @return The value of the absolute encoder in meters with the offset applied.
   */
  default double getAbsoluteHeightMetersOffset() {
    return 0.0;
  }

  /**
   * Gets the reading of the absolute encoder with offset.
   *
   * @return The value of the absolute encoder in meters with the offset applied.
   */
  default double getAbsoluteHeightMetersNoOffset() {
    return 0.0;
  }

  @FunctionalInterface
  interface IOFactory {
    ElevatorIO create(CCMotorController elevatorLeft, CCMotorController elevatorRight);
  }
}

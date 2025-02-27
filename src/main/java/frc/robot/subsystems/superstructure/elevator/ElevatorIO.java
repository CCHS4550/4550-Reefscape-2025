package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem.ElevatorState;
import frc.util.motorcontroller.CCMotorController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  class ElevatorIOInputs {
    public double currentPositionMeters;
    public double currentVelocityMetersPerSecond;

    public double currentRotationsTop;
    public double currentRotationsBottom;

    public boolean hallEffectTripped;

    public double pidOutput;
    public double ffOutput;
    public double appliedVoltage;

    public double pidError;

    public double setpointAngleDegrees;
    public double setpointAngleRadians;
    public double setpointVelocity;

    public double goalAngleRadians;
    public double goalAngleDegrees;
    public double goalVelocity;
  }

  /** Update the data coming into the robot. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void holdAtState(ElevatorState goalState) {}

  public default void holdAtStateWithVelocity(ElevatorState goalState, double velocity) {}

  public default Command goToGoalState(State goalState, ElevatorSubsystem elevator) {
    return new InstantCommand();
  }

  public default void setVoltage(Voltage voltage) {}

  // public default double getVoltage() {
  //   return 0.0;
  // }

  public default double getPIDFFOutput(State goalState) {
    return 0.0;
  }

  public default void stop() {}

  public default BooleanSupplier atSetpoint() {
    return () -> false;
  }

  default double getHeightAxleRotations() {
    return 0.0;
  }

  /**
   * Gets the reading of the relative encoder with offset.
   *
   * @return The value of the relative encoder in meters with the offset applied.
   */
  default double getHeightMeters() {
    return 0.0;
  }

  default double getVelocityMetersPerSecond() {
    return 0.0;
  }

  default void resetEncoder() {}

  default void resetPID() {}

  @FunctionalInterface
  interface IOFactory {
    ElevatorIO create(CCMotorController elevatorLeft, CCMotorController elevatorRight);
  }
}

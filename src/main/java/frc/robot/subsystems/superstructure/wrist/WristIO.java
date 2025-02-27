package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.wrist.WristSubsystem.WristState;
import frc.util.motorcontroller.CCMotorController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

  @AutoLog
  class WristIOInputs {
    public double currentRotations;

    public double currentAngleDegrees;
    public double currentAngleRadians;

    public double currentVelocity;

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

  // This might cause a problem, look here if Robot State is called prematurely.
  default double getAbsoluteEncoderGlobalRadians() {
    return getAbsoluteEncoderRadiansOffset()
        - RobotState.getInstance().armInputs.currentAngleRadians;
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

  default Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return new InstantCommand();
  }

  default Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return new InstantCommand();
  }

  default void resetPID() {}

  @FunctionalInterface
  interface IOFactory {
    WristIO create(CCMotorController motor);
  }
}

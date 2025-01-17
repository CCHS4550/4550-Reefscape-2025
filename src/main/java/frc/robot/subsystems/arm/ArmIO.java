package frc.robot.subsystems.Arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.helpers.CCMotorController;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
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
  public default void updateInputs(ArmIOInputsAutoLogged inputs) {}

  public default void holdAtState(ArmState goalState) {}

  public default Command goToGoalState(State goalState, ArmSubsystem arm) {
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

  // public default void setP(double p) {}

  // public default void setI(double i) {}

  // public default void setD(double d) {}

  // public default void setFF(double ff) {}

  // public default void setkS(double kS) {}

  // public default void setkV(double kV) {}

  // public default void setkG(double kG) {}

  // public default void setkA(double kA) {}

  // public default double getP() { return 0.0; }

  // public default double getI() { return 0.0; }

  // public default double getD() { return 0.0; }

  // public default double getFF() { return 0.0; }

  // public default double getkS() { return 0.0; }

  // public default double getkG() { return 0.0; }

  // public default double getkV() { return 0.0; }

  // public default double getkA() { return 0.0; }

  @FunctionalInterface
  interface IOFactory {
    ArmIO create(CCMotorController motor);
  }
}

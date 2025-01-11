package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Voltage;
import frc.helpers.CCMotorController;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  class ElevatorIOInputs {
    public double elevatorPosition;
  }

  /** Update the data coming into the robot. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(Voltage voltage) {}

  public default double getAngleDegrees() {
    return 0.0;
  }

  public default void stop() {}

  public default void goToSetPoint(double setpoint) {}

  public default void holdSetpoint(double setpoint) {}

  public default void changeElevatorPosition(ElevatorPositions desiredPosition){}

  public default boolean atSetpoint() {
    return false;
  }

  @FunctionalInterface
  interface IOFactory {
    ElevatorIO create(CCMotorController motor);
  }
}

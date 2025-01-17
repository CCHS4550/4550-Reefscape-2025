package frc.robot.subsystems.wrist;

import edu.wpi.first.units.measure.Voltage;
import frc.helpers.CCMotorController;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

  @AutoLog
  class WristIOInputs {
    public double wristAngleRads;
    public double wristAppliedVolts;
    public double wristCurrentDrawAmps;
    public double wristAngularMomentum; // rads per second

    public double intakeAppliedVolts;
    public double intakeCurrentDrawAmps;
  }

  /** Update the data coming into the robot. */
  public default void updateInputs(WristIOInputs inputs) {}

  public default void setVoltage(Voltage voltage) {}

  public default double getAngleDegrees() {
    return 0.0;
  }

  public default void stop() {}

  public default void goToSetPoint(double setpoint) {}

  public default void holdSetpoint(double setpoint) {}

  public default boolean atSetpoint() {
    return false;
  }

  @FunctionalInterface
  interface IOFactory {
    WristIO create(CCMotorController motor);
  }
}

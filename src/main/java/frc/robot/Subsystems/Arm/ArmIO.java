package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Voltage;
import frc.helpers.CCMotorController;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double currentAngleDegrees = 0.0;
    public double currentAngleRadians = 0.0;
    public double appliedVoltage = 0.0;
    public double setpointAngleDegrees = 0.0;
    public double setpointAngleRadians = 0.0;
  }

  /** Update the data coming into the robot. */
  public default void updateInputs(ArmIOInputs inputs) {}

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

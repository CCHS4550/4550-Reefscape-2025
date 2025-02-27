package frc.robot.subsystems.algae;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import frc.helpers.motorcontroller.CCMotorController;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeStates;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {

  @AutoLog
  class AlgaeIOInputs {

    public double currentAngleDegrees;
    public double currentAngleRadians;

    public double pidOutput;
    public double ffOutput;
    public double appliedVoltage;

    public double setpointAngleDegrees;
    public double setpointAngleRadians;
    public double setpointVelocity;

    public double goalAngleRadians;
    public double goalAngleDegrees;
    public double goalVelocity;
  }

  default void updateInputs(AlgaeIOInputs inputs) {}

  default void setTargetWristAngle(double Angle) {}

  default void setWristVoltage(Voltage voltage) {}

  default double getWristVoltage() {
    return 0.0;
  }

  default void setIntakeVoltage(Voltage voltage) {}

  default double getAngleRads() {
    return 0.0;
  }

  default void wristToStow() {}

  default void wristToIntake() {}

  default void holdAtState(AlgaeStates goalState) {}

  public default double getPIDFFOutput(State goalState) {
    return 0.0;
  }

  @FunctionalInterface
  interface IOFactory {
    AlgaeIO create(CCMotorController wrist, CCMotorController intake);
  }
}

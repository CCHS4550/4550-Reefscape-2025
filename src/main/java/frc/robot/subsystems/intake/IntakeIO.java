package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import frc.helpers.CCMotorController;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public double appliedInnerVoltage = 0.0;
    public double appliedOuterVoltagae = 0.0;

    public boolean hasCoral = false;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setOuterVoltage(Voltage voltage) {}

  default void setInnerVoltage(Voltage voltage) {}

  @FunctionalInterface
  interface IOFactory {
    IntakeIO create(CCMotorController innerMotor, CCMotorController outerMotor);
  }
}

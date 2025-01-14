package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import frc.helpers.CCMotorController;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {}

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setVoltage(Voltage voltage) {}
  
  default Command 

  @FunctionalInterface
  interface IOFactory {
    IntakeIO create(CCMotorController motor);
  }
}

package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Voltage;
import frc.helpers.CCMotorController;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  class ClimberIOInputs {}

  default void updateInputs(ClimberIOInputs inputs) {}

  default void winchDown() {}

  default void winchUp() {}

  default void setVoltage(Voltage voltage) {}

  @FunctionalInterface
  interface IOFactory {
    ClimberIO create(CCMotorController motor);
  }
}

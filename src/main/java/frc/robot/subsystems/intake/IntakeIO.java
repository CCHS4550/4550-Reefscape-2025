package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import frc.helpers.motorcontroller.CCMotorController;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {

    public double appliedVoltage = 0.0;

    public boolean hasCoral = false;
    public boolean beamBroke = false;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void intake(Voltage volts) {}

  default boolean hasCoral() {
    return false;
  }

  @FunctionalInterface
  interface IOFactory {
    IntakeIO create(CCMotorController intakeMotor);
  }
}

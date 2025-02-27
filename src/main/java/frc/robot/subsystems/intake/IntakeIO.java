package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.util.motorcontroller.CCMotorController;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {

    public double appliedVoltage;

    public boolean hasCoral;
    public boolean beamBroke;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void intake(Voltage volts) {}

  default Command intakeCommand(Voltage volts, IntakeSubsystem intake) {
    return new InstantCommand();
  }

  default boolean hasCoral() {
    return false;
  }

  @FunctionalInterface
  interface IOFactory {
    IntakeIO create(CCMotorController intakeMotor);
  }
}

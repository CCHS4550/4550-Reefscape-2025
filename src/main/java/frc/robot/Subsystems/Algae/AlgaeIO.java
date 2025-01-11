package frc.robot.Subsystems.Algae;

import edu.wpi.first.units.measure.Voltage;
import frc.helpers.CCMotorController;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {

  @AutoLog
  class AlgaeIOInputs {
    public double wristAngleRads;
    public double wristAppliedVolts;
    public double wristCurrentDrawAmps;
    public double wristAngularMomentum; // rads per second

    public double intakeAppliedVolts;
    public double intakeCurrentDrawAmps;
  }

  default void updateInputs(AlgaeIOInputs inputs) {}

  default void setTargetWristAngle(double Angle) {}

  default void setWristVoltage(Voltage voltage) {}

  default void setIntakeVoltage(Voltage voltage) {}

  default void wristToStow (){}

  default void wristToIntake(){}
  

  @FunctionalInterface
  interface IOFactory {
    AlgaeIO create(CCMotorController wrist, CCMotorController intake);
  }
}

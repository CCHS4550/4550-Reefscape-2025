package frc.robot.Subsystems.Algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
  default void updateInputs(AlgaeIOInputs inputs) {}

  default void setTargetWristAngle(double Angle) {}

  default void setWristvoltage(double voltage) {}

  default void setIntakeVoltage(double voltage) {}
}

@AutoLog
class AlgaeIOInputs {
  public double wristAngleRads;
  public double wristAppliedVolts;
  public double wristCurrentDrawAmps;
  public double wristAngularMomentum; // rads per second

  public double intakeAppliedVolts;
  public double intakeCurrentDrawAmps;
}

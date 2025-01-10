package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.wrist.WristIO;

public interface AlgaeIO {
  default void updateInputs(AlgaeIOInputs inputs) {}

  default void setTargetWristAngle(double Angle) {}

  default void setWristvoltage(double voltage) {}

  default void setIntakeVoltage(double voltage) {}




  @FunctionalInterface
  interface IOFactory {
    AlgaeIO create();
  }
  

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




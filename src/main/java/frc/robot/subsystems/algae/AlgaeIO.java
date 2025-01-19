package frc.robot.subsystems.algae;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.helpers.CCMotorController;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeStates;
import frc.robot.subsystems.arm.ArmSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {

  @AutoLog
  class AlgaeIOInputs {
    // public double wristAngleRads;
    // public double wristAppliedVolts;
    // public double wristCurrentDrawAmps;
    // public double wristAngularMomentum; // rads per second

    // public double intakeAppliedVolts;
    // public double intakeCurrentDrawAmps;
    public double currentAngleDegrees = 0.0;
    public double currentAngleRadians = 0.0;

    public double pidOutput = 0.0;
    public double ffOutput = 0.0;
    public double appliedVoltage = 0.0;

    public double setpointAngleDegrees = 0.0;
    public double setpointAngleRadians = 0.0;
    public double setpointVelocity = 0.0;

    public double goalAngleRadians = 0.0;
    public double goalAngleDegrees = 0.0;
    public double goalVelocity = 0.0;
  }

  default void updateInputs(AlgaeIOInputs inputs) {}

  default void setTargetWristAngle(double Angle) {}

  default void setWristVoltage(Voltage voltage) {}

  default double getWristVoltage(){
    return 0.0;
  }

  default void setIntakeVoltage(Voltage voltage) {}

  default double getAngleRads(){
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

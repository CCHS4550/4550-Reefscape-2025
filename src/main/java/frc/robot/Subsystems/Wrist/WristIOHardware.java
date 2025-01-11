package frc.robot.Subsystems.Wrist;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCMotorController;
import org.littletonrobotics.junction.Logger;

public class WristIOHardware implements WristIO {

  CCMotorController motor;

  private final ProfiledPIDController pidController;

  public WristIOHardware(CCMotorController motor) {
    this.motor = motor;

    pidController = new ProfiledPIDController(0, 0, 0, null);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage.magnitude());
  }

  /** SYSID METHODS */

  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the quasistatic characterization test
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the dynamic characterization test
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(1),
              Volts.of(5),
              Seconds.of(4),
              (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> setVoltage(voltage),
              null, // No log consumer, since data is recorded by URCL
              WristSubsystem.getInstance()));
}

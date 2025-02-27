package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Voltage;
import frc.util.motorcontroller.CCMotorController;

public class ClimberIOHardware implements ClimberIO {

  CCMotorController winchMotor;

  public ClimberIOHardware(CCMotorController winchMotor) {
    this.winchMotor = winchMotor;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVoltage = winchMotor.getVoltage();
  }

  @Override
  public void winchDown() {
    winchMotor.setVoltage(12, 60);
  }

  @Override
  public void winchUp() {
    winchMotor.setVoltage(-12, 60);
  }

  @Override
  public void winchStop() {
    winchMotor.setVoltage(0);
  }

  @Override
  public void setVoltage(Voltage volts) {
    winchMotor.setVoltage(volts.magnitude(), 60);
  }
}

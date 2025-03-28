package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Voltage;
import frc.util.motorcontroller.CCMotorController;

public class ClimberIOSim implements ClimberIO {

  CCMotorController winchMotor;

  public ClimberIOSim(CCMotorController winchMotor) {
    this.winchMotor = winchMotor;
  }

  @Override
  public void winchDown() {
    winchMotor.setVoltage(3, 60);
  }

  @Override
  public void winchUp() {
    winchMotor.setVoltage(-3, 60);
  }

  @Override
  public void setVoltage(Voltage volts) {
    winchMotor.setVoltage(volts.magnitude(), 60);
  }
}

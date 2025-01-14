package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import frc.helpers.CCMotorController;

public class IntakeIOHardware implements IntakeIO {

  CCMotorController motor;

  public IntakeIOHardware(CCMotorController motor) {
    this.motor = motor;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage.magnitude());
  }

  
}

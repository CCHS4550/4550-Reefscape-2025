package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import frc.helpers.CCMotorController;

public class IntakeIOHardware implements IntakeIO {

  CCMotorController innerMotor;
  CCMotorController outerMotor;

  public IntakeIOHardware(CCMotorController innerMotor, CCMotorController outerMotor) {
    this.innerMotor = innerMotor;
    this.outerMotor = outerMotor;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedInnerVoltage = innerMotor.getVoltage();
    inputs.appliedOuterVoltagae = outerMotor.getVoltage();
  }

  @Override
  public void setInnerVoltage(Voltage voltage) {
    innerMotor.setVoltage(voltage.magnitude());
  }

  @Override
  public void setOuterVoltage(Voltage voltage) {
    outerMotor.setVoltage(voltage.magnitude());
  }

  @Override
  public void setAllVoltage(Voltage voltage) {
    innerMotor.setVoltage(voltage.magnitude());
    outerMotor.setVoltage(voltage.magnitude());
  }
}

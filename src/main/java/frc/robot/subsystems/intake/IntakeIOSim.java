package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import frc.helpers.motorcontroller.CCMotorController;

public class IntakeIOSim implements IntakeIO {

  CCMotorController innerMotor;
  CCMotorController outerMotor;
  private final AnalogInputSim beamBreak;

  public IntakeIOSim(CCMotorController innerMotor, CCMotorController outerMotor) {
    this.innerMotor = innerMotor;
    this.outerMotor = outerMotor;
    beamBreak = new AnalogInputSim(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedInnerVoltage = innerMotor.getVoltage();
    inputs.appliedOuterVoltagae = outerMotor.getVoltage();
    inputs.beamBreakVoltage = beamBreak.getVoltage();
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

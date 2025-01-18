package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import frc.helpers.CCMotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

  public void setCollectiveVoltage(Voltage voltage){
    setInnerVoltage(voltage);
    setOuterVoltage(voltage);
  }

  public Command intake (){
    return this.runEnd(()-> setCollectiveVoltage (2.69), () -> setCollectiveVoltage(0));
  }

  public Command intakeForTime (double time){
    return intake().withTimeout(time);
  }
}

package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.helpers.maps.Constants;
import frc.helpers.motorcontroller.CCMotorController;

public class IntakeIOHardware implements IntakeIO {

  private CCMotorController intakeMotor;
  private DigitalInput beamBreak;

  public IntakeIOHardware(CCMotorController motor) {
    this.intakeMotor = motor;
    beamBreak = new DigitalInput(Constants.IntakeConstants.BEAM_BREAK_PORT);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVoltage = intakeMotor.getVoltage();

    inputs.beamBroke = !beamBreak.get();
    inputs.hasCoral = inputs.beamBroke;
  }

  @Override
  public void intake(Voltage volts) {
    intakeMotor.setVoltage(volts.magnitude());
  }

  @Override
  public boolean hasCoral() {
    return !beamBreak.get();
  }
}

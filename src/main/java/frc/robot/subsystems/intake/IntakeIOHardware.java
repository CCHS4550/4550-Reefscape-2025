package frc.robot.subsystems.intake;

import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.util.maps.Constants;
import frc.util.motorcontroller.CCMotorController;

public class IntakeIOHardware implements IntakeIO {

  private CCMotorController intakeMotor;
  private DigitalInput beamBreak;
  private DigitalOutput beamBreakPower;

  public IntakeIOHardware(CCMotorController motor) {
    this.intakeMotor = motor;
    beamBreak = new DigitalInput(Constants.IntakeConstants.BEAM_BREAK_PORT);
    beamBreakPower = new DigitalOutput(Constants.IntakeConstants.BEAM_BREAK_PORT_POWER);
    beamBreakPower.set(true);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVoltage = intakeMotor.getVoltage();

    inputs.beamBroke = !beamBreak.get();
    inputs.hasCoral = inputs.beamBroke;
  }

  @Override
  public void intake(Voltage volts) {
    // if (RobotState.getInstance().currentSuperState == SuperState.L4_BACK) {
    //   intakeMotor.setVoltage(-volts.magnitude());
    //   return;
    // }
    intakeMotor.setVoltage(volts.magnitude());
    return;
  }

  public Command intakeCommand(Voltage volts, IntakeSubsystem intake) {
    return startEnd(
        () -> intakeMotor.setVoltage(volts.magnitude()), () -> intakeMotor.setVoltage(0), intake);
  }

  @Override
  public boolean hasCoral() {
    return !beamBreak.get();
  }
}

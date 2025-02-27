package frc.robot.subsystems.intake;

import frc.util.motorcontroller.CCMotorController;

public class IntakeIOSim implements IntakeIO {

  CCMotorController intakeMotor;

  public IntakeIOSim(CCMotorController motor) {
    this.intakeMotor = motor;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}
}

package frc.robot.Subsystems.Algae;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.helpers.CCMotorController;

public class AlgaeIOHardware implements AlgaeIO {

  private static final double WRIST_GEAR_RATIO = 1.0;
  private static final double WRIST_POSITION_COEFFICIENT =
      (2 * Math.PI) * (WRIST_GEAR_RATIO * 2048); // idk why we muliply by 2048 but we do
  private static final double WRIST_VELOCITY_COEFFICIENT = WRIST_POSITION_COEFFICIENT * 10;

  public static final double WRIST_SLOW_ACCELERATION = Units.degreesToRadians(500);
  public static final double WRIST_FAST_ACCELERATION = Units.degreesToRadians(750);
  public static final double WRIST_VELOCITY = Units.degreesToRadians(300);

  private static final double WRIST_SLOW_ACCELERATION_CONSTRAINT =
      WRIST_SLOW_ACCELERATION / WRIST_VELOCITY_COEFFICIENT;
  private static final double WRIST_FAST_ACCELERATION_CONSTRAINT =
      WRIST_FAST_ACCELERATION / WRIST_VELOCITY_COEFFICIENT;
  private static final double SHOULDER_VELOCITY_CONSTRAINT =
      WRIST_VELOCITY / WRIST_VELOCITY_COEFFICIENT;

  private final CCMotorController wrist;
  private final CCMotorController intake;
  private ProfiledPIDController wristProfiledController;

  public AlgaeIOHardware(CCMotorController wrist, CCMotorController intake) {
    this.wrist = wrist;
    this.intake = intake;
    wristProfiledController = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(WRIST_VELOCITY, WRIST_FAST_ACCELERATION_CONSTRAINT));
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    // inputs.wristAngleRads = wrist.getRawPosition() * WRIST_POSITION_COEFFICIENT;
    // inputs.wristAppliedVolts = wrist.getBusVoltage() * wrist.getAppliedOutput(); //might just be
    // get bus voltage
    // inputs.wristCurrentDrawAmps = wrist.getOutputCurrent();
    // inputs.wristAngularMomentum = wrist.getRawVelocity(); // probably have to do some math to
    // this one, not in same units as example code

    // inputs.intakeAppliedVolts = intake.getBusVoltage() * intake.getAppliedOutput();
    // inputs.intakeCurrentDrawAmps = intake.getOutputCurrent();
  }

  @Override
  public void setWristVoltage(Voltage voltage) {
    wrist.setVoltage(voltage.magnitude());
  }

  @Override
  public void setIntakeVoltage(Voltage voltage) {
    intake.setVoltage(voltage.magnitude());
  }
@Override
  public void wristToStow(){
    // wristProfiledController(wrist.getPosition())
  }

  
}

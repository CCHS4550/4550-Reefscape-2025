package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.helpers.maps.Constants;
import frc.helpers.motorcontroller.CCMotorController;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeStates;

public class AlgaeIOSim implements AlgaeIO {

  private final CCMotorController wrist;
  private final CCMotorController intake;
  private PIDController wristPIDController;
  State goalState;
  ProfiledPIDController algaePidController;

  ArmFeedforward algaeFeedForward;

  double pidOutput;
  double ffOutput;

  public AlgaeIOSim(CCMotorController wrist, CCMotorController intake) {
    this.wrist = wrist;
    this.intake = intake;
    algaePidController =
        new ProfiledPIDController(
            Constants.ElevatorConstants.elevatorKP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.elevatorMaxVelocity,
                Constants.ElevatorConstants.elevatorMaxAcceleration));

    algaePidController.reset(0.0);
    // TODO Sysid
    algaeFeedForward = new ArmFeedforward(0, 0, 0, 0);

    goalState = new State(0, 0);
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
    inputs.currentAngleDegrees = Units.radiansToDegrees(getAngleRads());
    inputs.currentAngleRadians = getAngleRads();

    inputs.pidOutput = this.pidOutput;
    inputs.ffOutput = this.ffOutput;

    inputs.appliedVoltage = getWristVoltage();

    inputs.setpointAngleRadians = algaePidController.getSetpoint().position;
    inputs.setpointAngleDegrees = Units.radiansToDegrees(algaePidController.getSetpoint().position);
    inputs.setpointVelocity = algaePidController.getSetpoint().velocity;

    inputs.goalAngleRadians = goalState.position;
    inputs.goalAngleDegrees = Units.radiansToDegrees(goalState.position);
    inputs.goalVelocity = goalState.velocity;
  }

  @Override
  public void holdAtState(AlgaeStates goalState) {
    setWristVoltage(
        Volts.of(getPIDFFOutput(new State(Units.degreesToRadians(goalState.getAngle()), 0))));
  }

  @Override
  public double getPIDFFOutput(State goalState) {

    this.goalState = goalState;

    pidOutput = algaePidController.calculate(getAngleRads(), goalState);
    ffOutput =
        algaeFeedForward.calculate(
            algaePidController.getSetpoint().position, algaePidController.getSetpoint().velocity);

    return pidOutput + ffOutput;
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
  public double getWristVoltage() {
    return wrist.getVoltage();
  }

  @Override
  public void wristToStow() {
    wrist.set(
        wristPIDController.calculate(
            wrist.getPosition(), AlgaeSubsystem.AlgaeStates.STOW.getAngle()));
  }

  @Override
  public void wristToIntake() {
    wrist.set(
        wristPIDController.calculate(
            wrist.getPosition(), AlgaeSubsystem.AlgaeStates.STOW.getAngle()));
  }

  @Override
  public double getAngleRads() {
    return wrist.getPosition();
  }
}

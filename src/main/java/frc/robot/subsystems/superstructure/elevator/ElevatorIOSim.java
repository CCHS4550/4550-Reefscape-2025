package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.helpers.maps.Constants;
import frc.helpers.motorcontroller.CCMotorController;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem.ElevatorState;
import java.util.function.BooleanSupplier;

public class ElevatorIOSim implements ElevatorIO {

  CCMotorController elevatorLeft;
  CCMotorController elevatorRight;

  SparkMaxAlternateEncoderSim throughBore;

  ProfiledPIDController elevatorPidController;

  ElevatorFeedforward elevatorFeedForward;

  double pidOutput;
  double ffOutput;

  State goalState;

  public ElevatorIOSim(CCMotorController elevatorLeft, CCMotorController elevatorRight) {
    this.elevatorLeft = elevatorLeft;
    this.elevatorRight = elevatorRight;

    throughBore = (SparkMaxAlternateEncoderSim) elevatorLeft.getAlternateEncoder();

    elevatorPidController =
        new ProfiledPIDController(
            Constants.ElevatorConstants.elevatorKP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.elevatorMaxVelocity,
                Constants.ElevatorConstants.elevatorMaxAcceleration));

    elevatorPidController.reset(throughBore.getPosition());
    // TODO Sysid
    elevatorFeedForward = new ElevatorFeedforward(0, 0, 0, 0);

    goalState = new State(0, 0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.currentPositionMeters = getHeightMeters();

    inputs.pidOutput = this.pidOutput;
    inputs.ffOutput = this.ffOutput;

    inputs.setpointAngleRadians = elevatorPidController.getSetpoint().position;
    inputs.setpointAngleDegrees =
        Units.radiansToDegrees(elevatorPidController.getSetpoint().position);
    inputs.setpointVelocity = elevatorPidController.getSetpoint().velocity;

    inputs.goalAngleRadians = goalState.position;
    inputs.goalAngleDegrees = Units.radiansToDegrees(goalState.position);
    inputs.goalVelocity = goalState.velocity;
  }

  @Override
  public void holdAtState(ElevatorState goalState) {
    setVoltage(
        Volts.of(getPIDFFOutput(new State(Units.degreesToRadians(goalState.getHeight()), 0))));
  }

  public Command goToGoalState(State goalState, ElevatorSubsystem elevator) {
    return new FunctionalCommand(
        () -> {},
        () -> setVoltage(Volts.of(getPIDFFOutput(goalState))),
        (end) -> stop(),
        atSetpoint(),
        elevator);
  }

  /** Called continuously */
  @Override
  public double getPIDFFOutput(State goalState) {

    this.goalState = goalState;

    pidOutput = elevatorPidController.calculate(getHeightMeters(), goalState);
    ffOutput = elevatorFeedForward.calculate(elevatorPidController.getSetpoint().velocity);

    return pidOutput + ffOutput;
  }

  @Override
  public BooleanSupplier atSetpoint() {
    return () -> Math.abs(getHeightMeters()) <= 5.0;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    elevatorLeft.setVoltage(voltage.magnitude());
    elevatorRight.setVoltage(voltage.magnitude());
  }

  /**
   * Gets the reading of the absolute encoder with offset. Used for getting the offset.
   *
   * @return The value of the absolute encoder in radians without the offset applied.
   */
  @Override
  public double getHeightMeters() {
    return throughBore.getPosition()
        * Constants.ElevatorConstants.HEIGHT_METERS_PER_ELEVATOR_MOTOR_ROTATIONS;
  }
}

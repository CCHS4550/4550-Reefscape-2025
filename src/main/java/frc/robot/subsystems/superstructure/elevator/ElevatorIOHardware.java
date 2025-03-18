package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem.ElevatorState;
import frc.util.maps.Constants;
import frc.util.motorcontroller.CCMotorController;
import java.util.function.BooleanSupplier;

public class ElevatorIOHardware implements ElevatorIO {

  CCMotorController elevatorBottom;
  CCMotorController elevatorTop;

  RelativeEncoder elevatorEncoderBottom;
  RelativeEncoder elevatorEncoderTop;

  DigitalInput hallEffect = new DigitalInput(Constants.ElevatorConstants.HALL_EFFECT_PORT);

  ProfiledPIDController elevatorPidController;
  ElevatorFeedforward elevatorFeedForward;

  double pidOutput;
  double ffOutput;

  State goalState;

  public ElevatorIOHardware(CCMotorController elevatorBottom, CCMotorController elevatorTop) {
    this.elevatorBottom = elevatorBottom;
    this.elevatorTop = elevatorTop;

    elevatorEncoderBottom = (RelativeEncoder) elevatorBottom.getEncoder();
    elevatorEncoderTop = (RelativeEncoder) elevatorTop.getEncoder();

    elevatorEncoderBottom.setPosition(0);
    elevatorEncoderTop.setPosition(0);

    elevatorPidController =
        new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(30, 35));

    // elevatorPidController.setIntegratorRange(-3, 3);

    elevatorPidController.reset(getHeightMeters());
    pidOutput = 0;
    ffOutput = 0;

    elevatorFeedForward = new ElevatorFeedforward(.25, .60, 0, 0);

    goalState = new State(0, 0);

    SmartDashboard.putData("Elevator PID Controller", elevatorPidController);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.currentPositionMeters = getHeightMeters();
    inputs.currentVelocityMetersPerSecond = getVelocityMetersPerSecond();

    inputs.currentRotationsTop = elevatorEncoderTop.getPosition();
    inputs.currentRotationsBottom = elevatorEncoderBottom.getPosition();

    inputs.hallEffectTripped = !hallEffect.get();

    inputs.pidOutput = this.pidOutput;
    inputs.ffOutput = this.ffOutput;

    inputs.appliedVoltage = elevatorBottom.getVoltage();

    inputs.pidError = elevatorPidController.getPositionError();

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
        Volts.of(
            getPIDFFOutput(
                new State(
                    goalState.getHeight()
                        / Constants.ElevatorConstants.HEIGHT_METERS_TO_AXLE_ROTATIONS,
                    0))));
  }

  public void holdAtStateWithVelocity(ElevatorState goalState, double velocity) {
    setVoltage(
        Volts.of(
            getPIDFFOutput(
                new State(
                    goalState.getHeight()
                        / Constants.ElevatorConstants.HEIGHT_METERS_TO_AXLE_ROTATIONS,
                    velocity))));
  }

  public Command goToGoalState(State goalState, ElevatorSubsystem elevator) {
    return new FunctionalCommand(
        () -> {},
        () -> setVoltage(Volts.of(getPIDFFOutput(goalState))),
        (end) -> stop(),
        atSetpoint(),
        elevator);
  }

  @Override
  public void resetEncoders() {
    elevatorEncoderBottom.setPosition(0);
    elevatorEncoderTop.setPosition(0);
    resetPID();
  }

  /** Called continuously */
  @Override
  public double getPIDFFOutput(State goalState) {

    this.goalState = goalState;

    pidOutput = elevatorPidController.calculate(getHeightAxleRotations(), goalState);
    ffOutput = elevatorFeedForward.calculate(elevatorPidController.getSetpoint().velocity);

    return pidOutput + ffOutput;
  }

  @Override
  public BooleanSupplier atSetpoint() {
    return () -> Math.abs(getHeightMeters()) <= 5.0;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    elevatorBottom.setVoltage(voltage.magnitude());
    elevatorTop.setVoltage(voltage.magnitude());
  }

  @Override
  public double getHeightAxleRotations() {
    return ((elevatorEncoderBottom.getPosition() + elevatorEncoderTop.getPosition()) / 2)
        * Constants.ElevatorConstants.AXLE_ROTATIONS_TO_ELEVATOR_MOTOR_ROTATIONS;
  }

  /**
   * Gets the reading of the absolute encoder with offset. Used for getting the offset.
   *
   * @return The value of the absolute encoder in radians without the offset applied.
   */
  @Override
  public double getHeightMeters() {
    // System.out.println(Constants.ElevatorConstants.HEIGHT_METERS_PER_ELEVATOR_MOTOR_ROTATIONS);
    // System.out.println(Constants.ElevatorConstants.AXLE_ROTATIONS_TO_ELEVATOR_MOTOR_ROTATIONS);
    // System.out.println(Constants.ElevatorConstants.HEIGHT_METERS_TO_AXLE_ROTATIONS);
    return ((elevatorEncoderBottom.getPosition() + elevatorEncoderTop.getPosition()) / 2)
        * Constants.ElevatorConstants.HEIGHT_METERS_PER_ELEVATOR_MOTOR_ROTATIONS;
  }

  @Override
  public double getVelocityMetersPerSecond() {
    return ((elevatorEncoderBottom.getPosition() + elevatorEncoderTop.getPosition()) / 2)
        * Constants.ElevatorConstants.ELEVATOR_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR;
  }

  @Override
  public void resetPID() {
    elevatorPidController.reset(getHeightMeters());
    pidOutput = 0;
    ffOutput = 0;
  }
}

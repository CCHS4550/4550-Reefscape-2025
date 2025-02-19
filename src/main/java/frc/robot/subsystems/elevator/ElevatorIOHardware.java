package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.helpers.maps.Constants;
import frc.helpers.motorcontroller.CCMotorController;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import java.util.function.BooleanSupplier;

public class ElevatorIOHardware implements ElevatorIO {

  CCMotorController elevatorBottom;
  CCMotorController elevatorTop;

  RelativeEncoder elevatorEncoderBottom;
  RelativeEncoder elevatorEncoderTop;

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

    // throughBore = (RelativeEncoder) elevatorBottom.getAlternateEncoder();

    elevatorPidController =
        new ProfiledPIDController(
            Constants.ElevatorConstants.elevatorKP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.elevatorMaxVelocity,
                Constants.ElevatorConstants.elevatorMaxAcceleration));

    elevatorPidController.reset(getAbsoluteHeightMetersOffset());
    // TODO Sysid
    elevatorFeedForward = new ElevatorFeedforward(0, 0, 0, 0);

    goalState = new State(0, 0);

    SmartDashboard.putData("Elevator PID Controller", elevatorPidController);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.currentPositionRotations = Units.radiansToDegrees(getAbsoluteHeightMetersOffset());

    inputs.pidOutput = this.pidOutput;
    inputs.ffOutput = this.ffOutput;

    inputs.appliedVoltage = getVoltage();

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

    pidOutput = elevatorPidController.calculate(getAbsoluteHeightMetersOffset(), goalState);
    ffOutput = elevatorFeedForward.calculate(elevatorPidController.getSetpoint().velocity);

    return pidOutput + ffOutput;
  }

  @Override
  public BooleanSupplier atSetpoint() {
    return () -> Math.abs(getAbsoluteHeightMetersOffset()) <= 5.0;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    elevatorBottom.setVoltage(voltage.magnitude());
    // elevatorTop.setVoltage(voltage.magnitude());
  }

  @Override
  public double getAbsoluteHeightMetersOffset() {
    return (getAbsoluteHeightMetersNoOffset()
            * Constants.ElevatorConstants.AXLE_ROTATION_TO_HEIGHT_METERS)
        - Constants.ElevatorConstants.ELEVATOR_THROUGHBORE_OFFSET;
  }

  /**
   * Gets the reading of the absolute encoder with offset. Used for getting the offset.
   *
   * @return The value of the absolute encoder in radians without the offset applied.
   */
  @Override
  public double getAbsoluteHeightMetersNoOffset() {
    return ((elevatorEncoderBottom.getPosition() + elevatorEncoderTop.getPosition()) / 2)
        * Constants.ElevatorConstants.AXLE_ROTATION_TO_HEIGHT_METERS;
  }

  @Override
  public void resetPID() {
    elevatorPidController.reset(getAbsoluteHeightMetersOffset());
  }
}

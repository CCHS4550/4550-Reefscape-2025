package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.helpers.CCMotorController;
import frc.maps.Constants;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import java.util.function.BooleanSupplier;

public class ElevatorIOHardware implements ElevatorIO {

  CCMotorController elevatorLeft;
  CCMotorController elevatorRight;

  RelativeEncoder throughBore;

  ProfiledPIDController elevatorPidController;

  ElevatorFeedforward elevatorFeedForward;

  double pidOutput;
  double ffOutput;

  State goalState;

  public ElevatorIOHardware(CCMotorController elevatorLeft, CCMotorController elevatorRight) {
    this.elevatorLeft = elevatorLeft;
    this.elevatorRight = elevatorRight;

    throughBore = (RelativeEncoder) elevatorLeft.getAlternateEncoder();

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
    elevatorLeft.setVoltage(voltage.magnitude());
    elevatorRight.setVoltage(voltage.magnitude());
  }

  @Override
  public double getAbsoluteHeightMetersOffset() {
    return (throughBore.getPosition() * Constants.ElevatorConstants.AXLE_ROTATION_TO_HEIGHT_METERS)
        - Constants.ElevatorConstants.ELEVATOR_THROUGHBORE_OFFSET;
  }

  /**
   * Gets the reading of the absolute encoder with offset. Used for getting the offset.
   *
   * @return The value of the absolute encoder in radians without the offset applied.
   */
  @Override
  public double getAbsoluteHeightMetersNoOffset() {
    return throughBore.getPosition() * Constants.ElevatorConstants.AXLE_ROTATION_TO_HEIGHT_METERS;
  }

  /** SYSID METHODS */

  // /**
  //  * Used only in characterizing. Don't touch this.
  // //  *
  //  * @param direction
  //  * @return the quasistatic characterization test
  //  */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return sysIdRoutine.quasistatic(direction);
  // }

  // /**
  //  * Used only in characterizing. Don't touch this.
  //  *
  //  * @param direction
  //  * @return the dynamic characterization test
  //  */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return sysIdRoutine.dynamic(direction);
  // }

  // SysIdRoutine sysIdRoutine =
  //     new SysIdRoutine(
  //         new SysIdRoutine.Config(
  //             Volts.per(Second).of(1),
  //             Volts.of(5),
  //             Seconds.of(4),
  //             (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
  //         new SysIdRoutine.Mechanism(
  //             (voltage) -> setVoltage(voltage),
  //             null, // No log consumer, since data is recorded by URCL
  //             ElevatorSubsystem.getInstance()));
}

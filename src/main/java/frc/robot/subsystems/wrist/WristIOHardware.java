package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.helpers.CCMotorController;
import frc.maps.Constants;
import frc.robot.subsystems.wrist.WristSubsystem.WristState;
import java.util.function.BooleanSupplier;

public class WristIOHardware implements WristIO {

  CCMotorController wristMotor;

  RelativeEncoder throughBore;

  ProfiledPIDController wristPidController;
  ArmFeedforward wristFeedForward;

  double pidOutput;
  double ffOutput;

  State goalState;

  public WristIOHardware(CCMotorController wristMotor) {
    this.wristMotor = wristMotor;

    throughBore = (RelativeEncoder) wristMotor.getEncoder();

    wristPidController =
        new ProfiledPIDController(
            0, 0, 0, new TrapezoidProfile.Constraints(0, 0)); // do something for this

    wristPidController.reset(throughBore.getPosition());
    // TODO Sysid
    wristFeedForward = new ArmFeedforward(0, 0, 0, 0);

    goalState = new State(0, 0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    inputs.currentAngleDegrees = Units.radiansToDegrees(getAbsoluteEncoderRadiansOffset());
    inputs.currentAngleRadians = getAbsoluteEncoderRadiansOffset();

    inputs.pidOutput = this.pidOutput;
    inputs.ffOutput = this.ffOutput;

    inputs.appliedVoltage = getVoltage();

    inputs.setpointAngleRadians = wristPidController.getSetpoint().position;
    inputs.setpointAngleDegrees = Units.radiansToDegrees(wristPidController.getSetpoint().position);
    inputs.setpointVelocity = wristPidController.getSetpoint().velocity;

    inputs.goalAngleRadians = goalState.position;
    inputs.goalAngleDegrees = Units.radiansToDegrees(goalState.position);
    inputs.goalVelocity = goalState.velocity;
  }

  @Override
  public void holdAtState(WristState goalState) {
    setVoltage(
        Volts.of(getPIDFFOutput(new State(Units.degreesToRadians(goalState.getAngle()), 0))));
  }

  public Command goToGoalState(State goalState, WristSubsystem wrist) {
    return new FunctionalCommand(
        () -> {},
        () -> setVoltage(Volts.of(getPIDFFOutput(goalState))),
        (end) -> stop(),
        atSetpoint(),
        wrist);
  }

  /** Called continuously */
  @Override
  public double getPIDFFOutput(State goalState) {

    this.goalState = goalState;

    pidOutput = wristPidController.calculate(getAbsoluteEncoderRadiansOffset(), goalState);
    ffOutput =
        wristFeedForward.calculate(
            wristPidController.getSetpoint().position, wristPidController.getSetpoint().velocity);

    return pidOutput + ffOutput;
  }

  @Override
  public BooleanSupplier atSetpoint() {
    return () -> Math.abs(getAbsoluteEncoderRadiansOffset()) <= 5.0;
  }

  /**
   * Gets the reading of the absolute encoder with offset.
   *
   * @return The value of the absolute encoder in radians with the offset applied.
   */

  //  MAKE 0 PARALLEL OFF THE GROUND; STANDARD UNIT CIRCLE NOTATION.
  @Override
  public double getAbsoluteEncoderRadiansOffset() {
    return (throughBore.getPosition())
        - Constants.WristConstants.WRIST_THROUGHBORE_OFFSET
        + Math.PI;
    // return (throughBore.getPosition())
    //     - Constants.WristConstants.WRIST_THROUGHBORE_OFFSET
    //     + Math.PI;
  }

  /**
   * Gets the reading of the absolute encoder with offset. Used for getting the offset.
   *
   * @return The value of the absolute encoder in radians without the offset applied.
   */
  @Override
  public double getAbsoluteEncoderRadiansNoOffset() {
    return (throughBore.getPosition());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    wristMotor.setVoltage(voltage.magnitude());
  }

  @Override
  public double getVoltage() {
    return wristMotor.getVoltage();
  }

  @Override
  public void stop() {
    wristMotor.setVoltage(0);
  }

  /** SYSID METHODS */

  // /**
  //  * Used only in characterizing. Don't touch this.
  //  *
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
  //             WristSubsystem.getInstance()));
}

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
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
import frc.robot.subsystems.wrist.WristSubsystem.WristState;
import java.util.function.BooleanSupplier;

public class WristIOHardware implements WristIO {

  CCMotorController wristMotor;

  AbsoluteEncoder throughBore;

  ProfiledPIDController wristPidController;
  ArmFeedforward wristFeedForward;

  double pidOutput;
  double ffOutput;

  State goalState;

  public WristIOHardware(CCMotorController wristMotor) {
    this.wristMotor = wristMotor;

    throughBore = (AbsoluteEncoder) wristMotor.getDataportAbsoluteEncoder();

    wristPidController =
        new ProfiledPIDController(
            .05, 0, 0, new TrapezoidProfile.Constraints(1, 1)); // do something for this

    wristPidController.reset(throughBore.getPosition());
    // TODO Sysid
    wristFeedForward = new ArmFeedforward(0, 0, 0, 0);

    goalState = new State(0, 0);

    SmartDashboard.putData("Wrist PID Controller", wristPidController);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    inputs.currentAngleDegrees = Units.radiansToDegrees(getAbsoluteEncoderGlobalRadians());
    inputs.currentAngleRadians = getAbsoluteEncoderGlobalRadians();

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
    // ffOutput =
    //     wristFeedForward.calculate(
    //         getAbsoluteEncoderRadiansOffset(),
    // wristPidController.getSetpoint().velocity);

    return pidOutput;
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
    return getAbsoluteEncoderRadiansNoOffset() - Constants.WristConstants.WRIST_THROUGHBORE_OFFSET;
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
    return (throughBore.getPosition() * .75 * 2 * Math.PI);
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
}

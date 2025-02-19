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
            3, .3, 0, new TrapezoidProfile.Constraints(30, 5)); // do something for this

    // kI .3
    double min = ((-2 * Math.PI) - Constants.WristConstants.WRIST_THROUGHBORE_OFFSET) * .75;
    double max = -Constants.WristConstants.WRIST_THROUGHBORE_OFFSET * .75;
    wristPidController.enableContinuousInput(min, max);

    wristPidController.reset(throughBore.getPosition());
    // TODO Sysid
    wristFeedForward = new ArmFeedforward(.25, 0.54, 0.31);

    goalState = new State(0, 0);

    SmartDashboard.putData("Wrist PID Controller", wristPidController);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.currentRotations = -throughBore.getPosition();

    inputs.currentAngleRadians = getAbsoluteEncoderRadiansOffset();
    inputs.currentAngleDegrees = Units.radiansToDegrees(getAbsoluteEncoderRadiansOffset());
    inputs.currentVelocity = -throughBore.getVelocity() * Math.PI * 2 / 60;

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
  public void holdAtState(WristState wristState) {
    setVoltage(Volts.of(getPIDFFOutput(new State(wristState.getAngle(), 0))));
  }

  public Command goToGoalState(State state, WristSubsystem wrist) {
    return new FunctionalCommand(
        () -> {},
        () -> setVoltage(Volts.of(getPIDFFOutput(state))),
        (end) -> stop(),
        atSetpoint(),
        wrist);
  }

  /** Called continuously */
  @Override
  public double getPIDFFOutput(State state) {

    this.goalState = state;

    pidOutput = wristPidController.calculate(getAbsoluteEncoderRadiansOffset(), state);
    ffOutput =
        wristFeedForward.calculate(
            getAbsoluteEncoderRadiansOffset(), wristPidController.getSetpoint().velocity);

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
    double value =
        getAbsoluteEncoderRadiansNoOffset() - Constants.WristConstants.WRIST_THROUGHBORE_OFFSET;

    return value;
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

    double rev = -throughBore.getPosition();
    // unwrap(rev);

    double radiansNoOffset = (rev * 2 * Math.PI);
    return radiansNoOffset * 0.75;
  }

  @Override
  public void resetPID() {
    wristPidController.reset(throughBore.getPosition());
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
}

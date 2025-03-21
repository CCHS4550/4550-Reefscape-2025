package frc.robot.subsystems.superstructure.arm;

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
import frc.robot.subsystems.superstructure.arm.ArmSubsystem.ArmState;
import frc.util.maps.Constants;
import frc.util.motorcontroller.CCMotorController;
import java.util.function.BooleanSupplier;

public class ArmIOHardware implements ArmIO {

  CCMotorController armMotor;
  AbsoluteEncoder throughBore;

  ProfiledPIDController armPidController;
  ArmFeedforward feedForward;

  double pidOutput;
  double ffOutput;

  State goalState;

  public ArmIOHardware(CCMotorController armMotor) {

    this.armMotor = armMotor;
    throughBore = (AbsoluteEncoder) armMotor.getDataportAbsoluteEncoder();

    armPidController =
        new ProfiledPIDController(
            7, 0, 0, new TrapezoidProfile.Constraints(20, 10)); // do something for this

    // kI .5
    double min = ((-2 * Math.PI) - Constants.ArmConstants.ARM_THROUGHBORE_OFFSET);
    double max = -Constants.ArmConstants.ARM_THROUGHBORE_OFFSET;
    armPidController.enableContinuousInput(min, max);

    // armPidController.setIntegratorRange(-3, 3);

    armPidController.reset(throughBore.getPosition());
    pidOutput = 0;
    ffOutput = 0;

    // feedForward = new ArmFeedforward(.15, 1.34, 1.2, 0.09);
    // feedForward = new ArmFeedforward(.15, 1.0, 1.2);
    feedForward = new ArmFeedforward(.15, .9, 0);

    goalState = new State(0, 0);

    SmartDashboard.putData("Arm PID Controller", armPidController);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.currentRotations = -throughBore.getPosition();

    inputs.currentAngleDegrees = Units.radiansToDegrees(getAbsoluteEncoderRadiansOffset());
    inputs.currentAngleRadians = getAbsoluteEncoderRadiansOffset();

    inputs.currentVelocity = -throughBore.getVelocity() * Math.PI * .0333;

    inputs.pidOutput = this.pidOutput;
    inputs.ffOutput = this.ffOutput;

    inputs.appliedVoltage = getVoltage();

    inputs.pidError = armPidController.getPositionError();

    inputs.setpointAngleRadians = armPidController.getSetpoint().position;
    inputs.setpointAngleDegrees = Units.radiansToDegrees(armPidController.getSetpoint().position);
    inputs.setpointVelocity = armPidController.getSetpoint().velocity;

    inputs.goalAngleRadians = goalState.position;
    inputs.goalAngleDegrees = Units.radiansToDegrees(goalState.position);
    inputs.goalVelocity = goalState.velocity;
  }

  @Override
  public void holdAtState(ArmState armState) {
    setVoltage(Volts.of(getPIDFFOutput(new State(armState.getAngle(), 0))));
  }

  public Command goToGoalState(State state, ArmSubsystem arm) {
    return new FunctionalCommand(
        () -> {},
        () -> setVoltage(Volts.of(getPIDFFOutput(state))),
        (end) -> stop(),
        atSetpoint(),
        arm);
  }

  /** Called continuously */
  @Override
  public double getPIDFFOutput(State state) {

    this.goalState = state;

    pidOutput = armPidController.calculate(getAbsoluteEncoderRadiansOffset(), state);
    ffOutput =
        feedForward.calculate(
            getAbsoluteEncoderRadiansOffset(), armPidController.getSetpoint().velocity);

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
        getAbsoluteEncoderRadiansNoOffset() - Constants.ArmConstants.ARM_THROUGHBORE_OFFSET;

    return value;
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
    return radiansNoOffset;
  }

  @Override
  public void resetPID() {
    // armPidController.reset(throughBore.getPosition());
    pidOutput = 0;
    ffOutput = 0;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    armMotor.setVoltage(voltage.magnitude());
  }

  @Override
  public double getVoltage() {
    return armMotor.getVoltage();
  }

  @Override
  public void stop() {
    armMotor.setVoltage(0);
  }
}

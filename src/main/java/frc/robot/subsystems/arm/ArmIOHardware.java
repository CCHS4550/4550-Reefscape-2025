package frc.robot.subsystems.arm;

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
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;
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

    System.err.println("ARM IN REAL");

    this.armMotor = armMotor;
    throughBore = (AbsoluteEncoder) armMotor.getDataportAbsoluteEncoder();

    armPidController =
        new ProfiledPIDController(
            3.5, 0, 0, new TrapezoidProfile.Constraints(1, .5)); // do something for this

    double min = ((-2 * Math.PI) - Constants.ArmConstants.ARM_THROUGHBORE_OFFSET);
    double max = -Constants.ArmConstants.ARM_THROUGHBORE_OFFSET;
    armPidController.enableContinuousInput(min, max);
    armPidController.reset(throughBore.getPosition());
    // TODO Sysid
    /** Have to find Ks */
    feedForward = new ArmFeedforward(.15, 1.34, 1.87, 0.09);

    goalState = new State(0, 0);

    SmartDashboard.putData("Arm PID Controller", armPidController);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.currentRotations = -throughBore.getPosition();

    inputs.currentAngleDegrees = Units.radiansToDegrees(getAbsoluteEncoderRadiansOffset());
    inputs.currentAngleRadians = getAbsoluteEncoderRadiansOffset();

    inputs.pidOutput = this.pidOutput;
    inputs.ffOutput = this.ffOutput;

    inputs.appliedVoltage = getVoltage();

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
  public void setVoltage(Voltage voltage) {
    // armMotor.setVoltage(voltage.magnitude());
  }

  @Override
  public double getVoltage() {
    return armMotor.getVoltage();
  }

  @Override
  public void stop() {
    armMotor.setVoltage(0);
  }

  private double prev = Double.NaN;
  private int offset = 0;

  public double unwrap(double value) {
    if (!Double.isNaN(prev)) {
      // Compute number of wraps using precise modular arithmetic
      double delta = value - prev;
      offset += Math.rint(delta); // More precise rounding method
    }

    return value + offset;
  }
}

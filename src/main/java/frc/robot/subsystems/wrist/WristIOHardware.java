package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
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

  double previousValue = 0.0;
  double currentValue = 0.0;

  public WristIOHardware(CCMotorController wristMotor) {
    this.wristMotor = wristMotor;

    throughBore = (AbsoluteEncoder) wristMotor.getDataportAbsoluteEncoder();

    wristPidController =
        new ProfiledPIDController(
            5, 0, 0, new TrapezoidProfile.Constraints(.25, .5)); // do something for this

    double min = ((-2 * Math.PI) - Constants.WristConstants.WRIST_THROUGHBORE_OFFSET) * .75;
    double max = -Constants.WristConstants.WRIST_THROUGHBORE_OFFSET * .75;
    wristPidController.enableContinuousInput(min, max);

    wristPidController.reset(throughBore.getPosition());
    // TODO Sysid
    wristFeedForward = new ArmFeedforward(.25, 0.54, 0.31, 0.03);

    goalState = new State(0, 0);

    SmartDashboard.putData("Wrist PID Controller", wristPidController);

    wrapTimer.reset();
    wrapTimer.start();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.currentRotations = -throughBore.getPosition();

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
  public void setVoltage(Voltage voltage) {
    // wristMotor.setVoltage(voltage.magnitude());
  }

  @Override
  public double getVoltage() {
    return wristMotor.getVoltage();
  }

  @Override
  public void stop() {
    wristMotor.setVoltage(0);
  }

  public static boolean sameSign(double a, double b) {
    return (a >= -.5 && b >= -0.5) || (a < -.5 && b < -.5);
  }

  private double prev = Double.NaN;
  private int offset = 0;

  Timer wrapTimer = new Timer();
  double currentTime = 0;
  double prevTime = 0;

  /** true means Top */
  /** false means Bottom */
  private boolean aboutToWrap = false;

  public double unwrap(double value) {

    currentTime = wrapTimer.get();
    if (currentTime - prevTime > 0.1) {
      prevTime = currentTime;
      prev = value;
    }

    if (Math.abs(value - prev) > .2 && value < -.9) offset += 1;
    if (Math.abs(value - prev) > .2 && value > -.1) offset += -1;

    // if (value > -0.1) aboutToWrap = true;
    // if (value < -0.9) aboutToWrap = false;

    // if (!sameSign(currentValue, previousValue))

    // if (!Double.isNaN(prev)) {
    //   // Compute number of wraps using precise modular arithmetic
    //   double delta = value - prev;
    //   offset += Math.rint(delta);
    // }

    // prev = value;
    return value + offset;
  }

  /** SYSID METHODS */
}

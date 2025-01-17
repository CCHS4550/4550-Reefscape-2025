package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCMotorController;
import frc.maps.Constants;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ArmIOHardware implements ArmIO {

  CCMotorController motor;
  RelativeEncoder throughBore;

  ProfiledPIDController pidController;
  ArmFeedforward feedForward;

  double pidOutput;
  double ffOutput;

  State goalState;

  public ArmIOHardware(CCMotorController motor) {
    this.motor = motor;
    throughBore = (RelativeEncoder) motor.getAlternateEncoder();

    pidController =
        new ProfiledPIDController(
            0, 0, 0, new TrapezoidProfile.Constraints(0, 0)); // do something for this

    pidController.reset(throughBore.getPosition());
    // TODO Sysid
    feedForward = new ArmFeedforward(0, 0, 0, 0);

    goalState = new State(0, 0);
  }

  @Override
  public void updateInputs(ArmIOInputsAutoLogged inputs) {
    inputs.currentAngleDegrees = Units.radiansToDegrees(getAbsoluteEncoderRadiansOffset());
    inputs.currentAngleRadians = getAbsoluteEncoderRadiansOffset();

    inputs.pidOutput = this.pidOutput;
    inputs.ffOutput = this.ffOutput;

    inputs.appliedVoltage = getVoltage();

    inputs.setpointAngleRadians = pidController.getSetpoint().position;
    inputs.setpointAngleDegrees = Units.radiansToDegrees(pidController.getSetpoint().position);
    inputs.setpointVelocity = pidController.getSetpoint().velocity;

    inputs.goalAngleRadians = goalState.position;
    inputs.goalAngleDegrees = Units.radiansToDegrees(goalState.position);
    inputs.goalVelocity = goalState.velocity;
  }

  @Override
  public void holdAtState(ArmState goalState) {
    setVoltage(
        Volts.of(getPIDFFOutput(new State(Units.degreesToRadians(goalState.getAngle()), 0))));
  }

  public Command goToGoalState(State goalState, ArmSubsystem arm) {
    return new FunctionalCommand(
        () -> {},
        () -> setVoltage(Volts.of(getPIDFFOutput(goalState))),
        (end) -> stop(),
        atSetpoint(),
        arm);
  }

  /** Called continuously */
  @Override
  public double getPIDFFOutput(State goalState) {

    this.goalState = goalState;

    pidOutput = pidController.calculate(getAbsoluteEncoderRadiansOffset(), goalState);
    ffOutput =
        feedForward.calculate(
            pidController.getSetpoint().position, pidController.getSetpoint().velocity);

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
    return Units.rotationsToRadians(throughBore.getPosition())
        - Constants.SensorMiscConstants.ARM_THROUGHBORE_OFFSET
        + Math.PI;
  }

  /**
   * Gets the reading of the absolute encoder with offset. Used for getting the offset.
   *
   * @return The value of the absolute encoder in radians without the offset applied.
   */
  @Override
  public double getAbsoluteEncoderRadiansNoOffset() {
    return Units.rotationsToRadians(throughBore.getPosition());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage.magnitude());
  }

  @Override
  public double getVoltage() {
    return motor.getVoltage();
  }

  @Override
  public void stop() {
    motor.setVoltage(0);
  }

  /** SYSID METHODS */

  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the quasistatic characterization test
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the dynamic characterization test
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(1),
              Volts.of(5),
              Seconds.of(4),
              (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> setVoltage(voltage),
              null, // No log consumer, since data is recorded by URCL
              WristSubsystem.getInstance()));
}

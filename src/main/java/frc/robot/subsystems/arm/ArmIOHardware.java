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
import frc.helpers.CCMotorController;
import frc.maps.Constants;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;
import java.util.function.BooleanSupplier;

public class ArmIOHardware implements ArmIO {

  CCMotorController armMotor;
  AbsoluteEncoder throughBore;

  ProfiledPIDController pidController;
  ArmFeedforward feedForward;

  double pidOutput;
  double ffOutput;

  State goalState;

  public ArmIOHardware(CCMotorController armMotor) {

    System.err.println("ARM IN REAL");

    this.armMotor = armMotor;
    throughBore = (AbsoluteEncoder) armMotor.getDataportAbsoluteEncoder();

    pidController =
        new ProfiledPIDController(
            5, 0, .1, new TrapezoidProfile.Constraints(.5, .25)); // do something for this

    pidController.reset(throughBore.getPosition());
    // TODO Sysid
    feedForward = new ArmFeedforward(0, 0, 0, 0);

    goalState = new State(0, 0);

    SmartDashboard.putData("Arm PID Controller", pidController);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

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
    // ffOutput =
    //     feedForward.calculate(
    //         pidController.getSetpoint().position, pidController.getSetpoint().velocity);

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

    return getAbsoluteEncoderRadiansNoOffset() - Constants.ArmConstants.ARM_THROUGHBORE_OFFSET;
  }

  /**
   * Gets the reading of the absolute encoder with offset. Used for getting the offset.
   *
   * @return The value of the absolute encoder in radians without the offset applied.
   */
  @Override
  public double getAbsoluteEncoderRadiansNoOffset() {
    // System.out.println(throughBore.getPosition());
    return (armMotor.getPosition() * 2 * Math.PI);
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

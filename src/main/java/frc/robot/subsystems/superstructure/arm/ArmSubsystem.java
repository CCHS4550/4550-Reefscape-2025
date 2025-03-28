// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.util.maps.Constants;
import frc.util.motorcontroller.CCMotorController;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

  public enum ArmState {
    // Placeholder Values
    ZERO(0),
    DEFAULT_WITHINFRAME(Units.degreesToRadians(-36)),
    L1_FRONT(Units.degreesToRadians(0)),
    L2_FRONT(Units.degreesToRadians(13)),
    L3_FRONT(Units.degreesToRadians(17)),
    // L4_BACK(Units.degreesToRadians(94.234611)),
    L4_BACK(Units.degreesToRadians(91)),
    // 11.5
    CORAL_STATION_FRONT(Units.degreesToRadians(11)),
    CORAL_STATION_BACK(Units.degreesToRadians(0)),
    KNOCK_ALGAE_TOP(Units.degreesToRadians(-3)),
    KNOCK_ALGAE_BOTTOM(Units.degreesToRadians(-3)),
    CLIMB_PREPARING(Units.degreesToRadians(94.234611)),
    MANUAL(0); // this will never get applied
    public final double angleRadians;

    ArmState(double angleRadians) {
      this.angleRadians = angleRadians;
    }

    public double getAngle() {
      return angleRadians;
    }
  }

  public ArmState previousState = ArmState.DEFAULT_WITHINFRAME;
  public ArmState currentState = ArmState.DEFAULT_WITHINFRAME;
  public ArmState wantedState = ArmState.DEFAULT_WITHINFRAME;

  private final ArmIO armIO;

  private final SysIdRoutine sysIdRoutine;

  public final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  /** Creates a new WristSubsystem. */
  public ArmSubsystem(CCMotorController.MotorFactory motorFactory, ArmIO.IOFactory ioFactory) {

    this.armIO =
        ioFactory.create(
            motorFactory.create(
                "armMotor",
                "arm",
                Constants.MotorConstants.ARM,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.ARM_REVERSE,
                Constants.ArmConstants.ARM_MOTOR_ROTATIONS_TO_ARM_ROTATIONS_RADIANS,
                Constants.ArmConstants.ARM_MOTOR_RADIANS_PER_SECOND_CONVERSION_FACTOR));

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1),
                Volts.of(2),
                Seconds.of(2),
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> armIO.setVoltage(voltage),
                null, // No log consumer, since data is recorded by URCL
                this));
  }

  private void applyStates() {
    switch (currentState) {
      case ZERO:
        armIO.holdAtState(ArmState.ZERO);
        break;

      case DEFAULT_WITHINFRAME:
        armIO.holdAtState(ArmState.DEFAULT_WITHINFRAME);
        break;

      case L1_FRONT:
        armIO.holdAtState(ArmState.L1_FRONT);

        break;

      case L2_FRONT:
        armIO.holdAtState(ArmState.L2_FRONT);
        break;
      case L3_FRONT:
        armIO.holdAtState(ArmState.L3_FRONT);
        break;

      case L4_BACK:
        armIO.holdAtState(ArmState.L4_BACK);
        break;

      case CORAL_STATION_FRONT:
        armIO.holdAtState(ArmState.CORAL_STATION_FRONT);
        break;

      case CORAL_STATION_BACK:
        armIO.holdAtState(ArmState.CORAL_STATION_BACK);
        break;
      case KNOCK_ALGAE_TOP:
        armIO.holdAtState(ArmState.KNOCK_ALGAE_TOP);
        break;
      case KNOCK_ALGAE_BOTTOM:
        armIO.holdAtState(ArmState.KNOCK_ALGAE_BOTTOM);
        break;

      case MANUAL:
        break;

      default:
        armIO.holdAtState(ArmState.DEFAULT_WITHINFRAME);
        break;
    }
  }

  private ArmState handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case ZERO:
        return ArmState.ZERO;

      case DEFAULT_WITHINFRAME:
        return ArmState.DEFAULT_WITHINFRAME;

      case L1_FRONT:
        return ArmState.L1_FRONT;

      case L2_FRONT:
        return ArmState.L2_FRONT;

      case L3_FRONT:
        return ArmState.L3_FRONT;

      case L4_BACK:
        return ArmState.L4_BACK;

      case CORAL_STATION_FRONT:
        return ArmState.CORAL_STATION_FRONT;

      case CORAL_STATION_BACK:
        return ArmState.CORAL_STATION_BACK;
      case KNOCK_ALGAE_TOP:
        return ArmState.KNOCK_ALGAE_TOP;
      case KNOCK_ALGAE_BOTTOM:
        return ArmState.KNOCK_ALGAE_BOTTOM;
      case MANUAL:
        return ArmState.MANUAL;
      default:
        return ArmState.DEFAULT_WITHINFRAME;
    }
  }

  public void setWantedState(ArmState state) {
    this.wantedState = state;
  }

  public Command setWantedStateCommand(ArmState wantedState) {
    // System.out.println(wantedState);
    return new InstantCommand(() -> setWantedState(wantedState));
  }

  public ArmState getWantedState() {
    return wantedState;
  }

  public double getPosition() {
    return armIO.getAbsoluteEncoderRadiansOffset();
  }

  public Command testVoltageCommand(double volts) {
    return Commands.startEnd(
        () -> armIO.setVoltage(Volts.of(volts)), () -> armIO.setVoltage(Volts.of(0)));
  }

  public Command moveArmUpCommand() {
    return Commands.startEnd(
        () -> armIO.setVoltage(Volts.of(3)), () -> armIO.setVoltage(Volts.of(0)));
  }

  public Command moveArmDownCommand() {
    return Commands.startEnd(
        () -> armIO.setVoltage(Volts.of(-3)), () -> armIO.setVoltage(Volts.of(0)));
  }

  @Override
  public void periodic() {
    // System.out.println("currentState " + currentState);
    // System.out.println("wantedState " + wantedState);

    Logger.recordOutput("Subsystem/Arm/CurrentState", currentState.name());
    Logger.recordOutput("Subsystem/Arm/WantedState", wantedState.name());

    // System.out.println(getPosition());

    armIO.updateInputs(armInputs);
    Logger.processInputs("Subsystem/Arm", armInputs);

    if (RobotState.getInstance().allowSubsystemMovement.getAsBoolean()
        && RobotState.getInstance().moveArm.getAsBoolean()) {

      if (wantedState != currentState) {
        // armIO.resetPID();
        currentState = handleStateTransitions();
      }
      applyStates();
    }

    // // This method will be called once per scheduler run
  }

  public void resetPID() {
    armIO.resetPID();
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

  // /**
  //  * Used only in characterizing. Don't touch this.
  //  *
  //  * @param direction
  //  * @return the dynamic characterization test
  //  */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}

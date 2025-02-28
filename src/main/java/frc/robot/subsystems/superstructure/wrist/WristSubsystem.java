// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.util.maps.Constants;
import frc.util.motorcontroller.CCMotorController;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {

  public enum WristState {
    // Placeholder Values
    ZERO(0),
    DEFAULT_WITHINFRAME(Units.degreesToRadians(-188)),
    L1_FRONT(Units.degreesToRadians(15)),
    L2_FRONT(Units.degreesToRadians(75)),
    L3_FRONT(Units.degreesToRadians(75)),
    L4_BACK(Units.degreesToRadians(130)),
    CORAL_STATION_FRONT(Units.degreesToRadians(-135)),
    CORAL_STATION_BACK(Units.degreesToRadians(0)),
    CLIMB_PREPARING(Units.degreesToRadians(130));

    private final double angleRadians;

    WristState(double angleRadians) {
      this.angleRadians = angleRadians;
    }

    public double getAngle() {
      return angleRadians;
    }
  }

  public WristState previousState = WristState.DEFAULT_WITHINFRAME;
  public WristState currentState = WristState.DEFAULT_WITHINFRAME;
  public WristState wantedState = WristState.DEFAULT_WITHINFRAME;

  public final WristIO wristIO;

  private final SysIdRoutine sysIdRoutine;

  public final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

  /** Creates a new WristSubsystem. */
  public WristSubsystem(CCMotorController.MotorFactory motorFactory, WristIO.IOFactory ioFactory) {

    this.wristIO =
        ioFactory.create(
            motorFactory.create(
                "wristMotor",
                "wrist",
                Constants.MotorConstants.WRIST,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.WristConstants.WRIST_REVERSE,
                1,
                1));

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(.3),
                Volts.of(.4),
                Seconds.of(1),
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> wristIO.setVoltage(voltage),
                null, // No log consumer, since data is recorded by URCL
                this));
  }

  private void applyStates() {
    switch (currentState) {
      case ZERO:
        wristIO.holdAtState(WristState.ZERO);
        break;
      case DEFAULT_WITHINFRAME:
        wristIO.holdAtState(WristState.DEFAULT_WITHINFRAME);
        break;

      case L1_FRONT:
        wristIO.holdAtState(WristState.L1_FRONT);
        break;

      case L2_FRONT:
        wristIO.holdAtState(WristState.L2_FRONT);
        break;

      case L3_FRONT:
        wristIO.holdAtState(WristState.L3_FRONT);
        break;

      case L4_BACK:
        wristIO.holdAtState(WristState.L4_BACK);
        break;

      case CORAL_STATION_FRONT:
        wristIO.holdAtState(WristState.CORAL_STATION_FRONT);
        break;

      case CORAL_STATION_BACK:
        wristIO.holdAtState(WristState.CORAL_STATION_BACK);
        break;

      default:
        wristIO.holdAtState(WristState.DEFAULT_WITHINFRAME);
        break;
    }
  }

  private WristState handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case ZERO:
        return WristState.ZERO;
      case DEFAULT_WITHINFRAME:
        return WristState.DEFAULT_WITHINFRAME;

      case L1_FRONT:
        return WristState.L1_FRONT;

      case L2_FRONT:
        return WristState.L2_FRONT;

      case L3_FRONT:
        return WristState.L3_FRONT;

      case L4_BACK:
        return WristState.L4_BACK;

      case CORAL_STATION_FRONT:
        return WristState.CORAL_STATION_FRONT;

      case CORAL_STATION_BACK:
        return WristState.CORAL_STATION_BACK;

      default:
        return WristState.DEFAULT_WITHINFRAME;
    }
  }

  public Command runCharacterization() {
    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(wristIO.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    c.addCommands(wristIO.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    c.addCommands(wristIO.sysIdDynamic(SysIdRoutine.Direction.kForward));
    c.addCommands(wristIO.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    return c;
  }

  public void setWantedState(WristState wantedState) {
    this.wantedState = wantedState;
  }

  public Command setWantedStateCommand(WristState wantedSuperState) {
    return new InstantCommand(() -> setWantedState(wantedSuperState));
  }

  public WristState getWantedState() {
    return wantedState;
  }

  public Command wristUpCommand() {
    return Commands.startEnd(
        () -> wristIO.setVoltage(Volts.of(3)), () -> wristIO.setVoltage(Volts.of(0)));
  }

  public Command wristDownCommand() {
    return Commands.startEnd(
        () -> wristIO.setVoltage(Volts.of(-3)), () -> wristIO.setVoltage(Volts.of(0)));
  }
  // public double getGlobalPosition() {
  //   return wristIO.getAbsoluteEncoderRadiansOffset()
  //       - RobotState.getInstance().armInputs.currentAngleRadians;
  // }

  @Override
  public void periodic() {

    Logger.recordOutput("Subsystem/Wrist/CurrentState", currentState.name());
    Logger.recordOutput("Subsystem/Wrist/WantedState", wantedState.name());

    wristIO.updateInputs(wristInputs);
    Logger.processInputs("Subsystem/Wrist", wristInputs);

    if (RobotState.getInstance().allowSubsystemMovement.getAsBoolean()
        && RobotState.getInstance().moveWrist.getAsBoolean()) {

      if (wantedState != currentState) {
        // wristIO.resetPID();
        currentState = handleStateTransitions();
      }
      applyStates();
    }

    // This method will be called once per scheduler run
  }

  public void resetPID() {
    wristIO.resetPID();
  }

  public Command wristUp() {
    return this.startEnd(
        () -> {
          wristIO.setVoltage(Volts.of(5.0));
        },
        () -> {
          wristIO.setVoltage(Volts.of(0.0));
        });
  }

  public Command wristDown() {
    return this.startEnd(
        () -> {
          wristIO.setVoltage(Volts.of(-5.0));
        },
        () -> {
          wristIO.setVoltage(Volts.of(0.0));
        });
  }

  public Command testVoltageCommand(double volts) {
    return Commands.startEnd(
        () -> wristIO.setVoltage(Volts.of(volts)), () -> wristIO.setVoltage(Volts.of(0)));
  }
  /* SysID Factory Methods */

  // /**
  //  * Used only in characterizing. Don't touch this.
  // //  *
  //  * @param direction
  //  * @return the quasistatic characterization test
  //  */
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

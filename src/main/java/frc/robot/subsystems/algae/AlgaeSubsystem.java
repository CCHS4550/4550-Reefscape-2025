package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.maps.Constants;
import frc.helpers.motorcontroller.CCMotorController;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {

  private final AlgaeIO algaeIO;

  private final SysIdRoutine sysIdRoutine;

  public final AlgaeIOInputsAutoLogged algaeInputs = new AlgaeIOInputsAutoLogged();

  public enum AlgaeStates {
    INTAKE(Units.degreesToRadians(15)), // I have no idea what the right value is for this
    PROCESSOR(Units.degreesToRadians(30)), // I have no idea what the right value is for this
    STOW(Units.degreesToRadians(0));

    public final double angleRadians;

    AlgaeStates(double angleRadians) {
      this.angleRadians = angleRadians;
    }

    public double getAngle() {
      return angleRadians;
    }
  }

  public AlgaeStates previousState = AlgaeStates.STOW;
  public AlgaeStates currentState = AlgaeStates.STOW;
  public AlgaeStates wantedState = AlgaeStates.STOW;

  /** Creates a new WristSubsystem. */
  public AlgaeSubsystem(CCMotorController.MotorFactory motorFactory, AlgaeIO.IOFactory ioFactory) {

    this.algaeIO =
        ioFactory.create(
            motorFactory.create(
                "algaeWristMotor",
                "algaeWrist",
                Constants.MotorConstants.ALGAE_WRIST,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.ALGAE_WRIST_REVERSE,
                1.0,
                1.0),
            motorFactory.create(
                "algaeIntakeMotor",
                "algaeIntake",
                Constants.MotorConstants.ALGAE_INTAKE,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.ALGAE_WRIST_REVERSE,
                1.0,
                1.0));

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1),
                Volts.of(1),
                Seconds.of(2),
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> algaeIO.setWristVoltage(voltage),
                null, // No log consumer, since data is recorded by URCL
                this));
  }

  private void applyStates() {
    switch (currentState) {
      case STOW:
        algaeIO.holdAtState(AlgaeStates.STOW);
        break;
      case INTAKE:
        algaeIO.holdAtState(AlgaeStates.INTAKE);
        break;
      case PROCESSOR:
        algaeIO.holdAtState(AlgaeStates.PROCESSOR);
        break;
      default:
        algaeIO.holdAtState(AlgaeStates.STOW);
        break;
    }
  }

  private AlgaeStates handleStateTransitions() {
    previousState = currentState;
    switch (wantedState) {
      case STOW:
        return AlgaeStates.STOW;
      case INTAKE:
        return AlgaeStates.INTAKE;
      case PROCESSOR:
        return AlgaeStates.PROCESSOR;
      default:
        return AlgaeStates.STOW;
    }
  }

  public void setWantedState(AlgaeStates wantedState) {
    this.wantedState = wantedState;
  }

  public Command setWantedStateCommand(AlgaeStates wantedSuperState) {
    return new InstantCommand(() -> setWantedState(wantedSuperState));
  }

  public AlgaeStates getWantedState() {
    return wantedState;
  }

  @Override
  public void periodic() {
    algaeIO.updateInputs(algaeInputs);
    Logger.processInputs("Subsystem/Algae", algaeInputs);

    if (wantedState != currentState) currentState = handleStateTransitions();
    // applyStates();
  }
  // This method will be called once per scheduler run
  public Command wristUp() {
    return this.startEnd(
        () -> {
          algaeIO.setWristVoltage(Volts.of(5.0));
        }, // change this later
        () -> {
          algaeIO.setWristVoltage(Volts.of(0.0));
        });
  }

  public Command wristDown() {
    return this.startEnd(
        () -> {
          algaeIO.setWristVoltage(Volts.of(-5.0));
        }, // change this later
        () -> {
          algaeIO.setWristVoltage(Volts.of(0.0));
        });
  }

  public Command outtake() {
    return this.startEnd(
        () -> {
          algaeIO.setIntakeVoltage(Volts.of(5.0));
        }, // change this later
        () -> {
          algaeIO.setIntakeVoltage(Volts.of(0.0));
        });
  }

  public Command intake() {
    return this.startEnd(
        () -> {
          algaeIO.setIntakeVoltage(Volts.of(5.0));
        }, // change this later
        () -> {
          algaeIO.setIntakeVoltage(Volts.of(0.0));
        });
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
}

package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCMotorReplay;
import frc.maps.Constants;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {

  /** Implementation of Singleton Pattern */
  public static AlgaeSubsystem mInstance;

  private final AlgaeIO io;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCMotorReplay::new;
  private static AlgaeIO.IOFactory defaultIoFactory = AlgaeIOReplay::new;

  CCMotorController.MotorFactory motorFactory;
  AlgaeIO.IOFactory ioFactory;

  private final AlgaeIOInputsAutoLogged algaeInputs = new AlgaeIOInputsAutoLogged();

  public static AlgaeSubsystem getInstance(
      CCMotorController.MotorFactory motorFactory, AlgaeIO.IOFactory ioFactory) {
    if (mInstance == null) {
      mInstance = new AlgaeSubsystem(motorFactory, ioFactory);
    }
    return mInstance;
  }

  public static AlgaeSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new AlgaeSubsystem(defaultMotorFactory, defaultIoFactory);
      System.out.println("CREATING DEFAULT ALGAE");
    }
    return mInstance;
  }

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
  private AlgaeSubsystem(CCMotorController.MotorFactory motorFactory, AlgaeIO.IOFactory ioFactory) {
    this.motorFactory = motorFactory;
    this.ioFactory = ioFactory;

    this.io =
        ioFactory.create(
            this.motorFactory.create(
                "algaeWristMotor",
                "algaeWrist",
                Constants.MotorConstants.ALGAE_WRIST,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.ALGAE_WRIST_REVERSE,
                1.0,
                1.0),
            this.motorFactory.create(
                "algaeIntakeMotor",
                "algaeIntake",
                Constants.MotorConstants.ALGAE_INTAKE,
                MotorType.kBrushless,
                IdleMode.kBrake,
                Constants.MotorConstants.ALGAE_WRIST_REVERSE,
                1.0,
                1.0));
  }

  private void applyStates() {
    switch (currentState) {
      case STOW:
        io.holdAtState(AlgaeStates.STOW);
      case INTAKE:
        io.holdAtState(AlgaeStates.INTAKE);
      case PROCESSOR:
        io.holdAtState(AlgaeStates.PROCESSOR);
      default:
        io.holdAtState(AlgaeStates.STOW);
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
    io.updateInputs(algaeInputs);
    Logger.processInputs("Subsystem/Algae", algaeInputs);

    currentState = handleStateTransitions();
    applyStates();
  }
  // This method will be called once per scheduler run
  public Command wristUp() {
    return this.startEnd(
        () -> {
          io.setWristVoltage(Volts.of(5.0));
        }, // change this later
        () -> {
          io.setWristVoltage(Volts.of(0.0));
        });
  }

  public Command wristDown() {
    return this.startEnd(
        () -> {
          io.setWristVoltage(Volts.of(-5.0));
        }, // change this later
        () -> {
          io.setWristVoltage(Volts.of(0.0));
        });
  }

  public Command outtake() {
    return this.startEnd(
        () -> {
          io.setIntakeVoltage(Volts.of(5.0));
        }, // change this later
        () -> {
          io.setIntakeVoltage(Volts.of(0.0));
        });
  }

  public Command intake() {
    return this.startEnd(
        () -> {
          io.setIntakeVoltage(Volts.of(5.0));
        }, // change this later
        () -> {
          io.setIntakeVoltage(Volts.of(0.0));
        });
  }
}

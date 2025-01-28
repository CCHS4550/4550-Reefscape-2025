// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCMotorController;
import frc.helpers.CCMotorReplay;
import frc.maps.Constants;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {

  public enum WristState {
    // Placeholder Values
    ZERO(0),
    DEFAULT_WITHINFRAME(Units.degreesToRadians(0)),
    L1_FRONT(Units.degreesToRadians(30)),
    L2L3_FRONT(Units.degreesToRadians(45)),
    L4_BACK(Units.degreesToRadians(110)),
    CORAL_STATION_FRONT(Units.degreesToRadians(15)),
    CORAL_STATION_BACK(Units.degreesToRadians(120));

    private final double angleRadians;

    WristState(double angleRadians) {
      this.angleRadians = angleRadians;
    }

    public double getAngle() {
      return angleRadians;
    }
  }

  public static WristState previousState = WristState.DEFAULT_WITHINFRAME;
  public static WristState currentState = WristState.DEFAULT_WITHINFRAME;
  public static WristState wantedState = WristState.DEFAULT_WITHINFRAME;

  /** Implementation of Singleton Pattern */
  public static WristSubsystem mInstance;

  public final WristIO wristIO;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCMotorReplay::new;
  private static WristIO.IOFactory defaultIoFactory = WristIOReplay::new;

  CCMotorController.MotorFactory motorFactory;
  WristIO.IOFactory ioFactory;

  public static final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

  /** Singleton Practice */
  public static WristSubsystem getInstance(
      CCMotorController.MotorFactory motorFactory, WristIO.IOFactory ioFactory) {
    if (mInstance == null) {
      mInstance = new WristSubsystem(motorFactory, ioFactory);
    }
    return mInstance;
  }

  /** Singleton Practice */
  public static WristSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new WristSubsystem(defaultMotorFactory, defaultIoFactory);
      System.out.println("CREATING DEFAULT WRIST");
    }
    return mInstance;
  }

  /** Creates a new WristSubsystem. */
  private WristSubsystem(CCMotorController.MotorFactory motorFactory, WristIO.IOFactory ioFactory) {
    this.motorFactory = motorFactory;
    this.ioFactory = ioFactory;
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
  }

  private void applyStates() {
    switch (currentState) {
      case ZERO:
        wristIO.holdAtState(WristState.ZERO);
      case DEFAULT_WITHINFRAME:
        wristIO.holdAtState(WristState.DEFAULT_WITHINFRAME);

      case L1_FRONT:
        wristIO.holdAtState(WristState.L1_FRONT);

      case L2L3_FRONT:
        wristIO.holdAtState(WristState.L2L3_FRONT);

      case L4_BACK:
        wristIO.holdAtState(WristState.L4_BACK);

      case CORAL_STATION_FRONT:
        wristIO.holdAtState(WristState.CORAL_STATION_FRONT);

      case CORAL_STATION_BACK:
        wristIO.holdAtState(WristState.CORAL_STATION_BACK);

      default:
        wristIO.holdAtState(WristState.DEFAULT_WITHINFRAME);
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

      case L2L3_FRONT:
        return WristState.L2L3_FRONT;

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

  public double getPosition() {
    return wristIO.getAbsoluteEncoderRadiansOffset();
  }

  @Override
  public void periodic() {
    wristIO.updateInputs(wristInputs);
    Logger.processInputs("Subsystem/Wrist", wristInputs);
    currentState = handleStateTransitions();
    applyStates();

    // This method will be called once per scheduler run
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
}

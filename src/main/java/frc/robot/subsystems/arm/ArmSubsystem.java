// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

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

public class ArmSubsystem extends SubsystemBase {

  public enum ArmState {
    // Placeholder Values
    ZERO(0),
    DEFAULT_WITHINFRAME(Units.degreesToRadians(0)),
    L1_FRONT(Units.degreesToRadians(30)),
    L2L3_FRONT(Units.degreesToRadians(45)),
    L4_BACK(Units.degreesToRadians(110)),
    CORAL_STATION_FRONT(Units.degreesToRadians(15)),
    CORAL_STATION_BACK(Units.degreesToRadians(120));

    public final double angleRadians;

    ArmState(double angleRadians) {
      this.angleRadians = angleRadians;
    }

    public double getAngle() {
      return angleRadians;
    }
  }

  public static ArmState previousState;
  public static ArmState currentState;
  public static ArmState wantedState;

  /** Implementation of Singleton Pattern */
  public static ArmSubsystem mInstance;

  private final ArmIO armIO;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCMotorReplay::new;
  private static ArmIO.IOFactory defaultIoFactory = ArmIOReplay::new;

  CCMotorController.MotorFactory motorFactory;
  ArmIO.IOFactory ioFactory;

  private static ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  public static ArmSubsystem getInstance(
      CCMotorController.MotorFactory motorFactory, ArmIO.IOFactory ioFactory) {
    if (mInstance == null) {
      mInstance = new ArmSubsystem(motorFactory, ioFactory);
      System.out.println("CREATING ARM");
    }
    return mInstance;
  }

  public static ArmSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ArmSubsystem(defaultMotorFactory, defaultIoFactory);
      System.out.println("CREATING DEFAULT ARM");
    }
    return mInstance;
  }

  /** Creates a new WristSubsystem. */
  private ArmSubsystem(CCMotorController.MotorFactory motorFactory, ArmIO.IOFactory ioFactory) {
    this.motorFactory = motorFactory;
    this.ioFactory = ioFactory;
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

    previousState = ArmState.DEFAULT_WITHINFRAME;
    currentState = ArmState.DEFAULT_WITHINFRAME;
    wantedState = ArmState.DEFAULT_WITHINFRAME;
  }

  private void applyStates() {
    switch (currentState) {
      case ZERO:
        armIO.holdAtState(ArmState.ZERO);

      case DEFAULT_WITHINFRAME:
        armIO.holdAtState(ArmState.DEFAULT_WITHINFRAME);

      case L1_FRONT:
        armIO.holdAtState(ArmState.L1_FRONT);

      case L2L3_FRONT:
        armIO.holdAtState(ArmState.L2L3_FRONT);

      case L4_BACK:
        armIO.holdAtState(ArmState.L4_BACK);

      case CORAL_STATION_FRONT:
        armIO.holdAtState(ArmState.CORAL_STATION_FRONT);

      case CORAL_STATION_BACK:
        armIO.holdAtState(ArmState.CORAL_STATION_BACK);

      default:
        armIO.holdAtState(ArmState.DEFAULT_WITHINFRAME);
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

      case L2L3_FRONT:
        return ArmState.L2L3_FRONT;

      case L4_BACK:
        return ArmState.L4_BACK;

      case CORAL_STATION_FRONT:
        return ArmState.CORAL_STATION_FRONT;

      case CORAL_STATION_BACK:
        return ArmState.CORAL_STATION_BACK;

      default:
        return ArmState.DEFAULT_WITHINFRAME;
    }
  }

  public void setWantedState(ArmState state) {
    this.wantedState = state;
  }

  public Command setWantedStateCommand(ArmState wantedState) {
    System.out.println(wantedState);
    return new InstantCommand(() -> setWantedState(wantedState));
  }

  public ArmState getWantedState() {
    return wantedState;
  }

  public double getPosition() {
    return armIO.getAbsoluteEncoderRadiansOffset();
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

    currentState = handleStateTransitions();
    // currentState = ArmState.ZERO;
    applyStates();

    // This method will be called once per scheduler run
  }

  public Command armUp() {
    return this.startEnd(
        () -> {
          armIO.setVoltage(Volts.of(5.0));
        },
        () -> {
          armIO.setVoltage(Volts.of(0.0));
        });
  }

  public Command armDown() {
    return this.startEnd(
        () -> {
          armIO.setVoltage(Volts.of(-5.0));
        },
        () -> {
          armIO.setVoltage(Volts.of(0.0));
        });
  }
}

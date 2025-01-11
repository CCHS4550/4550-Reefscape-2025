package frc.robot.Subsystems.Algae;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class AlgaeSubsystem extends SubsystemBase {

  /** Implementation of Singleton Pattern */
  public static AlgaeSubsystem mInstance;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCSparkMax::new;
  private static AlgaeIO.IOFactory defaultIoFactory = AlgaeIOHardware::new;

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
    }
    return mInstance;
  }

  /** Creates a new WristSubsystem. */
  private AlgaeSubsystem(CCMotorController.MotorFactory motorFactory, AlgaeIO.IOFactory ioFactory) {
    this.motorFactory = motorFactory;
    this.ioFactory = ioFactory;
  }

  public final AlgaeIO io =
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
}

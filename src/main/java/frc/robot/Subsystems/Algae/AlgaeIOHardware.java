package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.maps.Constants;
import frc.helpers.CCSparkMax;

public class AlgaeIOHardware implements AlgaeIO {

  private static final double WRIST_GEAR_RATIO = 1.0;
  private static final double WRIST_POSITION_COEFFICIENT =
      (2 * Math.PI) * (WRIST_GEAR_RATIO * 2048); // idk why we muliply by 2048 but we do
  private static final double WRIST_VELOCITY_COEFFICIENT = WRIST_POSITION_COEFFICIENT * 10;

  public static final double WRIST_SLOW_ACCELERATION = Units.degreesToRadians(500);
  public static final double WRIST_FAST_ACCELERATION = Units.degreesToRadians(750);
  public static final double WRIST_VELOCITY = Units.degreesToRadians(300);

  private static final double WRIST_SLOW_ACCELERATION_CONSTRAINT =
      WRIST_SLOW_ACCELERATION / WRIST_VELOCITY_COEFFICIENT;
  private static final double SHOULDER_FAST_ACCELERATION_CONSTRAINT =
      WRIST_FAST_ACCELERATION / WRIST_VELOCITY_COEFFICIENT;
  private static final double SHOULDER_VELOCITY_CONSTRAINT =
      WRIST_VELOCITY / WRIST_VELOCITY_COEFFICIENT;

  private final CCSparkMax wrist;
  private final CCSparkMax intake;

  public AlgaeIOHardware(){
    wrist = new CCSparkMax("wrist", "wr", Constants.ALGAE_WRIST_ID, MotorType.kBrushless, IdleMode.kBrake, Constants.ALGAE_WRIST_REVERSED, 1.0, 1.0);
    intake = new CCSparkMax("algae intake", "an", Constants.ALGAE_INTAKE_ID, MotorType.kBrushless, IdleMode.kBrake, Constants.ALGAE_INTAKE_REVERSED, 1.0, 1.0);
  
  }
  @Override
  public void updateInputs(AlgaeIOInputs inputs){
    inputs.wristAngleRads = wrist.getRawPosition() * WRIST_POSITION_COEFFICIENT;
    inputs.wristAppliedVolts = wrist.getBusVoltage() * wrist.getAppliedOutput(); //might just be get bus voltage
    inputs.wristCurrentDrawAmps = wrist.getOutputCurrent();
    inputs.wristAngularMomentum = wrist.getRawVelocity(); // probably have to do some math to this one, not in same units as example code

    inputs.intakeAppliedVolts = intake.getBusVoltage() * intake.getAppliedOutput();
    inputs.intakeCurrentDrawAmps = intake.getOutputCurrent();
  }
  @Override
  public void setWristvoltage(double speed){
    wrist.setVoltageFromSpeed(speed);
  }
  @Override
  public void setIntakeVoltage(double speed){
    intake.setVoltageFromSpeed(speed);
  }
}

package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCMotorController;
import frc.helpers.CCSparkMax;
import frc.robot.subsystems.algae.AlgaeIOInputsAutoLogged;



public class AlgaeSubsystem extends SubsystemBase{

    /** Implementation of Singleton Pattern */
  public static AlgaeSubsystem mInstance;

  private static CCMotorController.MotorFactory defaultMotorFactory = CCSparkMax::new;
  private static AlgaeIO.IOFactory defaultIoFactory = AlgaeIOHardware::new;

  CCMotorController.MotorFactory motorFactory;
  AlgaeIO.IOFactory ioFactory;

  public static AlgaeSubsystem getInstance(CCMotorController.MotorFactory motorFactory, AlgaeIO.IOFactory ioFactory) {
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
    //add robot constants

    private final AlgaeIOInputsAutoLogged algaeInputs = new AlgaeIOInputsAutoLogged();

    
}

package frc.robot.subsystems.climber;

import frc.helpers.CCMotorController;

public interface ClimberIO {

    default void winchDown() {}

    default void winchUp() {}


    @FunctionalInterface
    interface IOFactory {
      ClimberIO create(CCMotorController motor);
    }
    
    
} 



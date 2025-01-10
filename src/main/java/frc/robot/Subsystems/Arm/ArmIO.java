package frc.robot.subsystems.arm;

public interface ArmIO {


    @FunctionalInterface
    interface IOFactory {
      ArmIO create();
    }
    
    
}



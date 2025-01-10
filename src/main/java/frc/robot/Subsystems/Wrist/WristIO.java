package frc.robot.subsystems.wrist;

public interface WristIO {



    @FunctionalInterface
    interface IOFactory {
      WristIO create();
    }
}

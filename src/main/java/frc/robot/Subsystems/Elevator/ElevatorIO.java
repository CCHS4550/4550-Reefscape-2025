package frc.robot.subsystems.elevator;

import frc.robot.subsystems.wrist.WristIO;

public interface ElevatorIO {

    @FunctionalInterface
    interface IOFactory {
        ElevatorIO create();
    }
}

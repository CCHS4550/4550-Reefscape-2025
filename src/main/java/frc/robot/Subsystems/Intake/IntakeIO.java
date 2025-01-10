package frc.robot.subsystems.intake;

import frc.robot.subsystems.wrist.WristIO;

public interface IntakeIO {

    @FunctionalInterface
    interface IOFactory {
      IntakeIO create();
    }
}

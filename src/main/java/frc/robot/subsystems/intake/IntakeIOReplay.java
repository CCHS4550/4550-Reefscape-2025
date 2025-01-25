// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.helpers.CCMotorController;

/** Add your docs here. */
public class IntakeIOReplay implements IntakeIO {
  public IntakeIOReplay(CCMotorController innerMotor, CCMotorController outerMotor) {
    System.err.println("INTAKE IN REPLAY");
  }
}

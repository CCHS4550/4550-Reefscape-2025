// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

/** Add your docs here. */
public interface SparkMaxBase {     


    default void set(double speed) {}
    
      default void setVoltage(double volts) {}
    
      default void setVoltage(double volts, double currentlimit) {}
    
      /* The actual speed in whatever units. */
      default double getVelocity() {
        return 0;
      }
    
      /* What you set it to. */
      default double getSpeed() {
        return 0;
      }

}

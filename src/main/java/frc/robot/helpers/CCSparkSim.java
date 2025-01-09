// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class CCSparkSim extends SparkMaxSim implements CCMotorController{
    public CCSparkSim() {
        super(SparkMax::new, );

    }
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.CCSparkMax;
public class Elevator extends SubsystemBase{
    CCSparkMax elevator1 = new CCSparkMax("elevator 1", "e1", Constants.MotorConstants.ELEVATOR_1, MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.ELEVATOR_1_REVERSED);
    CCSparkMax elevator2 = new CCSparkMax("elevator 2", "e2",  Constants.MotorConstants.ELEVATOR_2, MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.ELEVATOR_2_REVERSED);
    double height;

    he
    




}

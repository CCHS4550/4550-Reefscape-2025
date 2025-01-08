package frc.robot.subsystems;

import frc.robot.Constants;
import frc.helpers.CCSparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; // fuck yu

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Elevator extends SubsystemBase {
    CCSparkMax elevatorLeft = new CCSparkMax ("elevator 1", "e1", Constants.MotorConstants.ELEVATOR_ONE, MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.ELEVATOR_ONE_REVERSE);
    CCSparkMax elevatorRight = new CCSparkMax ("elevator 2", "e2", Constants.MotorConstants.ELEVATOR_TWO, MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.ELEVATOR_TWO_REVERSE);
    

}
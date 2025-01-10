package frc.robot.subsystems;
import frc.robot.Constants;
import frc.helpers.CCSparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class CoralCarriage extends SubsystemBase{
    CCSparkMax coralWrist = new CCSparkMax ("wrist", "w", Constants.MotorConstants.CORAL_WRIST, MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.CORAL_WRIST_REVERSE);
    PIDController wristPID = new PIDController (5, 0,0);
    // CCSparkMax indexer = new CCSparkMax
    
    public void setWristSetpoint(double setpoint){
        coralWrist.set(wristPID.calculate(coralWrist.getPosition(), setpoint));
    }
    public double getWristAngle(){
        double angle = Units.rotationsToRadians(coralWrist.getPosition()) % 2*Math.PI;
        return angle;
    }
    

}

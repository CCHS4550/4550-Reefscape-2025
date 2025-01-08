package frc.robot.subsystems;
import frc.robot.Constants;
import frc.helpers.CCSparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
public class CoralCarriage extends SubsystemBase{
    CCSparkMax coralWrist = new CCSparkMax ("wrist", "w", Constants.MotorConstants.CORAL_WRIST, MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.CORAL_WRIST_REVERSE);
    PIDController wristPID = new PIDController (5, 0,0);
    
    public void setWristSetpoint(double setpoint){
        wrist.set(wristPID.calculate(coralWrist.getPosition(), ))
    }
    public double getWristAngle(){
        double angle = Units.rotationsToRadians(coralWrist.getPosition()) % 2*Math.PI;
        return angle;
    }
}

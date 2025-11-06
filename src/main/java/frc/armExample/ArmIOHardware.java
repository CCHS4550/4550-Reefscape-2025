package frc.armExample;

import static edu.wpi.first.units.Units.Radian;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import frc.util.motorcontroller.CCMotorController;

public class ArmIOHardware implements ArmIO{
    CCMotorController armMotor;
    PIDController pidController;
    AbsoluteEncoder encoder;
    double desiredRadians;

    public ArmIOHardware (CCMotorController armMotor){
        this.armMotor = armMotor;
        encoder = (AbsoluteEncoder) armMotor.getDataportAbsoluteEncoder();

        pidController = new PIDController(67, 0, 0);

        
    }

    @Override
    public void updateInputs(ArmIOInputs io){
        io.appliedVolts = armMotor.getVoltage();

    }

    public void goToRadians(double desiredRadians){
        setGoal(desiredRadians);

        armMotor.setVoltage(pidController.calculate(desiredRadians));
    }

    public void setGoal(double desiredRadians){
        this.desiredRadians = desiredRadians;
        
    }

    public double getGoal(){
        return desiredRadians;
    }

    @Override
    public void setVoltage(double volts){
        armMotor.setVoltage(volts);
    }
    
}

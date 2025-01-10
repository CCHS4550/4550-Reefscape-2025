package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Voltage;
import frc.helpers.CCMotorController;
import frc.robot.subsystems.climber.ClimberIO;

public class ClimberIOHardware implements ClimberIO {

    CCMotorController winchMotor;


    public ClimberIOHardware(CCMotorController winchMotor) {
        this.winchMotor = winchMotor;

    }

    @Override
    public void winchDown() {
        winchMotor.setVoltage(3, 60);
    }

    @Override
    public void winchUp() {
        winchMotor.setVoltage(-3, 60);
    }

    
    public void setWinchVoltage(Voltage volts) {
        winchMotor.setVoltage(3, 60);
    }


}


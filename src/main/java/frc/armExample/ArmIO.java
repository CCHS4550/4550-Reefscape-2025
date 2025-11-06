package frc.armExample;

import frc.util.motorcontroller.CCMotorController;

public interface ArmIO {
    public class ArmIOInputs {
        public double appliedVolts;
        public double currentAngleRadians;
        public double desiredAngleRadians;
    }

    public default void updateInputs (ArmIOInputs io){}

    public default void setVoltage (double volts){}

    

}
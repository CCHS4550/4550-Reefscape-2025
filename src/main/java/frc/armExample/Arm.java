package frc.armExample;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private ArmIO io;

    public Arm(ArmIO io){
        this.io = io; 
    }

    @Override
    public void periodic (){
        io.updateInputs(io);
    }
}

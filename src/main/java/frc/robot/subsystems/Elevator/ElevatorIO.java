package frc.robot.subsystems.Elevator;
import org.littletonrobotics.*;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO{
    default void updateInputs(ElevatorIOInputs io ){}
    default void setStow() {}
    default void setL1(){}
    default void setL2(){}
    default void setL3(){}
    default void setL4(){}
    default void setA1(){}
    default void setA2(){}
    default void setProcessor(){}
    default void changeStates(){}
    default void rotationsToHeight(double rotations){}
    default void heightToRotations(double height){}

    @AutoLog
    class ElevatorIOInputs{
        // idk whatto put here
    }

}


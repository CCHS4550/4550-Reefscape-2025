package frc.robot.subsystems.Elevator;
import frc.robot.Constants;
import frc.helpers.CCSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Elevator extends SubsystemBase {
    private CCSparkMax elevatorLeft = new CCSparkMax("elevator left", "eL", Constants.MotorConstants.ELEVATOR_LEFT, MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.ELEVATOR_LEFT_REVERSE);
    private PIDController elevatorLeftController;
    private RelativeEncoder elevatorLeftEncoder;

    private CCSparkMax elevatorRight = new CCSparkMax("elevator right", "eR", Constants.MotorConstants.ELEVATOR_RIGHT, MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.ELEVATOR_RIGHT_REVERSE);
    private PIDController elevatorRightController;
    private RelativeEncoder elevatorRightEncoder;
    private Timer timer;

    // private TrapezoidProfile profile;
    // private TrapezoidProfile.State currentProfile ;
    // private TrapezoidProfile.State goalProfile ;
    public static Elevator instance;

    private ProfiledPIDController elevatorLeftProfiledPID;
    private ProfiledPIDController elevatorRightProfiledPID;

    public static Elevator getInstance (){
        if (instance == null){
            instance = new Elevator();
        }
        return instance;
    }
    public enum ElevatorStates{
        STOW (Constants.ElevatorConstants.elevatorHeights[0]),
        L1 (Constants.ElevatorConstants.elevatorHeights[1]),
        L2 (Constants.ElevatorConstants.elevatorHeights[2]), 
        L3 (Constants.ElevatorConstants.elevatorHeights[3]),
        L4 (Constants.ElevatorConstants.elevatorHeights[4]),
        A1 (Constants.ElevatorConstants.elevatorHeights[5]),
        A2 (Constants.ElevatorConstants.elevatorHeights[6]),
        PROCESSOR(Constants.ElevatorConstants.elevatorHeights[7]);
        public double height;
         ElevatorStates (double height){
            this.height = height;
        }
        public double getHeight(){
            return height;
        }
        
    }
    ElevatorStates currentState;


    public Elevator (){
        
        // profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.ElevatorConstants.elevatoreMaxVelocity, Constants.ElevatorConstants.elevatorMaxAcceleration));
        timer = new Timer();
        timer.start();
        elevatorLeftProfiledPID = new ProfiledPIDController(5,0,0, new TrapezoidProfile.Constraints(Constants.ElevatorConstants.elevatoreMaxVelocity, Constants.ElevatorConstants.elevatorMaxAcceleration));
        elevatorRightProfiledPID = new ProfiledPIDController(5,0,0, new TrapezoidProfile.Constraints(Constants.ElevatorConstants.elevatoreMaxVelocity, Constants.ElevatorConstants.elevatorMaxAcceleration));
    }
    public double rotationsToHeight(double rotations){
        return rotations * 0.3345 + Constants.ElevatorConstants.stowHeight; //random number
    }
    public double heightToRotations (double height){
        return height * 0.35234;
    }

    public void setStow(){
        currentState = ElevatorStates.STOW;
    }
    public void setL1 (){
        currentState = ElevatorStates.L1;
    }
    public void setL2(){
        currentState = ElevatorStates.L2;
    }
    public void setL3(){
        currentState = ElevatorStates.L3;
    }
    public void setL4(){
        currentState = ElevatorStates.L4;
    }
    public void setA1(){
        currentState = ElevatorStates.A1;
    }
    public void setA2(){
        currentState = ElevatorStates.A2;
    }
    public void setProcessor(){
        currentState = ElevatorStates.PROCESSOR;
    }
    public void setElevatorVoltage (double voltage){
        elevatorLeft.setVoltage(voltage);
        elevatorRight.setVoltage(voltage);
    }

    public void changeStates(){
        double currentHeight = rotationsToHeight(elevatorLeft.getPosition()) + Constants.MotorConstants.stowHeight;
        double desiredHeight = currentState.getHeight();
        elevatorLeft.set(elevatorLeftProfiledPID.calculate(heightToRotations(currentHeight), heightToRotations(desiredHeight)));
        elevatorRight.set(elevatorRightProfiledPID.calculate(heightToRotations(currentHeight), heightToRotations(desiredHeight)));
        
    }
    

}


package frc.robot.subsystems;

import frc.robot.Constants;
import frc.helpers.CCSparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; 
import edu.wpi.first.math.controller.ProfiledPIDController;


import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Elevator extends SubsystemBase {
    private CCSparkMax elevatorLeft;
    private PIDController elevatorLeftController;
    private RelativeEncoder elevatorLeftEncoder;

    private CCSparkMax elevatorRight;
    private PIDController elevatorRightController;
    private RelativeEncoder elevatorRightEncoder;
    private Timer timer;

    private TrapezoidProfile profile;
    private TrapezoidProfile.State currentProfile = new TrapezoidProfile();
    private TrapezoidProfile.State goalProfile = new TrapezoidProfile();
    public static Elevator instance;

    private ProfiledPIDController elevatorProfiledPID;

    public static Elevator getInstance (){
        if (instance == null){
            instance = new Elevator();
        }
        return instance;
    }
    public enum ElevatorStates{
        STOW (Constants.ElevatorConstants.elevatorHeights[0]),
        L1 (Constants.ElevatorConstants.elevatorHeights[1]),
        L2 (Constants.ElevatorConstants.elevatoreHeights[2]), 
        L3 (Constants.ElevatorConstants.elevatorHeights[3]),
        L4 (Constants.ElevatorConstants.elevatorHeights[4]),
        A1 (Constants.ElevatorConstants.elevatorHeights[5]),
        A2 (Constants.ElevatorConstants.elevatorHeights[6]),
        PROCESSOR(Constants.ElevatorConstants.elevatorHeights[7]);
        public double height;
        public ElevatorStates (double height){
            this.height = height;
        }
        public double getHeight(){
            return height;
        }
        
    }
    ElevatorStates currentState;


    public Elevator (CCSparkMax elevatorLeft, CCSparkMax elevatorRight){
        this.elevatorRight = elevatorRight;
        this.elevatorLeft = elevatorLeft;
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.ElevatorConstants.elevatorMaxVelocity, Constants.ElevatorConstants.elevatorMaxAcceleration));
        timer = new Timer();
        timer.start();
        elevatorProfiledPID = new ProfiledPIDController(5,0,0, new TrapezoidProfile.Constraints(Constants.ElevatorConstants.elevatoreMaxVelocity, Constants.ElevatorConstants.elevatorMaxAcceleration));
    }
    public double rotationsToHeight(double rotations){
        return rotations * 0.3345 + Constants.ElevatorConstants.heightStowed; //random number
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

    public void changeStates(){
        double currentHeight = rotationToHeight(elevatorLeft.getPosition());
        currentState.getHeight();
    }
    

}
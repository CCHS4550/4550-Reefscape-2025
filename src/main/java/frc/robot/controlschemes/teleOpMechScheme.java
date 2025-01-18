package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.algae.AlgaeIOHardware;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmIOHardware;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberIOHardware;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.wrist.WristIOHardware;
import frc.robot.subsystems.wrist.WristSubsystem;

public class teleOpMechScheme {
    private static CommandGenericHID teleOpBoard;
    
    
    
    public static void configure(IntakeSubsystem intake, ArmSubsystem arm, ElevatorSubsystem elevator, WristSubsystem wrist, AlgaeSubsystem algae, ClimberSubsystem climber,
    int port) {
        teleOpBoard = new CommandGenericHID(port);
        configureButtons( intake, arm, elevator, wrist, algae, climber, port);
        }
    public static void configureButtons(IntakeSubsystem intake, ArmSubsystem arm, ElevatorSubsystem elevator, WristSubsystem wrist, AlgaeSubsystem algae, ClimberSubsystem climber,
    int port){
        teleOpBoard.button(1).whileTrue(intake.intake());//intake
        teleOpBoard.button(2).whileTrue(intake.outtake());//outtake
    }
}

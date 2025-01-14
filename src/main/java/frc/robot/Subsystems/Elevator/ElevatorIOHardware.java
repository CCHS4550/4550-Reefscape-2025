package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCMotorController;
import frc.maps.Constants;
import frc.robot.subsystems.Wrist.WristSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPositions;

import org.littletonrobotics.junction.Logger;

public class ElevatorIOHardware implements ElevatorIO {

  CCMotorController elevatorLeft;
  CCMotorController elevatorRight;
  ProfiledPIDController elevatorLeftPidController;
  ProfiledPIDController elevatorRightPidController  ;

  public ElevatorIOHardware(CCMotorController elevatorLeft, CCMotorController elevatorRight) {
    this.elevatorLeft = elevatorLeft;
    this.elevatorRight = elevatorRight;

    elevatorLeftPidController = new ProfiledPIDController(Constants.ElevatorConstants.elevatorKP, 0, 0, new TrapezoidProfile.Constraints(Constants.ElevatorConstants.elevatorMaxVelocity, Constants.ElevatorConstants.elevatorMaxAcceleration));
    elevatorRightPidController = new ProfiledPIDController(Constants.ElevatorConstants.elevatorKP, Constants.ElevatorConstants.elevatorKI, Constants.ElevatorConstants.elevatorKD, new TrapezoidProfile.Constraints(Constants.ElevatorConstants.elevatorMaxVelocity, Constants.ElevatorConstants.elevatorMaxAcceleration));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    elevatorLeft.setVoltage(voltage.magnitude());
    elevatorRight.setVoltage(voltage.magnitude());
  }

  @Override
  public void changePosition (ElevatorPositions desiredPosition){
   double currentHeight = ElevatorSubsystem.rotationsToHeight(elevatorLeft.getPosition());
   elevatorLeftPidController.calculate(elevatorLeft.getPosition(), ElevatorSubsystem.heightToRotations(desiredPosition.getHeight()));
   elevatorRightPidController.calculate(elevatorRight.getPosition(), ElevatorSubsystem.heightToRotations(desiredPosition.getHeight()));

  }
  



  /** SYSID METHODS */

  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the quasistatic characterization test
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Used only in characterizing. Don't touch this.
   *
   * @param direction
   * @return the dynamic characterization test
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(1),
              Volts.of(5),
              Seconds.of(4),
              (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> setVoltage(voltage),
              null, // No log consumer, since data is recorded by URCL
              WristSubsystem.getInstance()));
}

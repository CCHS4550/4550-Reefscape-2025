package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import frc.helpers.CCSparkMax;
import frc.maps.RobotMap;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveModule;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCSparkMax;
import frc.robot.subsystems.*;



public class SwerveDrive {
    private Vision vision;
    private Timer timer;
    private PIDController chassisXSPidController;
    private PIDController chassisYSPidController;
    private PIDController chassisThetaPidController;
    private double currentTime;
    private SwerveModule[] swerveModules;
    private SwerveModuleState[] swerveModuleStates;
    private SwerveModulePosition[] swerveModulePositions;
    public static AHRS gyro;
    public static SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public static ChassisSpeeds chassisSpeeds;

    public SwerveDrive(Vision vision, ){
        this.vision = vision;
        public final SwerveModule frontRight =
    new SwerveModule(
        new CCSparkMax(
            "Front Right Drive",
            "frd",
            Constants.MotorConstants.FRONT_RIGHT_DRIVE,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.FRONT_RIGHT_DRIVE_REVERSE,
            Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
            Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
        new CCSparkMax(
            "Front Right Turn",
            "frt",
            Constants.MotorConstants.FRONT_RIGHT_TURN,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.FRONT_RIGHT_TURN_REVERSE,
            Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
            Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
        Constants.SwerveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER,
        Constants.SwerveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET,
        "Front Right");

public static final SwerveModule frontLeft =
    new SwerveModule(
        new CCSparkMax(
            "Front Left Drive",
            "fld",
            Constants.MotorConstants.FRONT_LEFT_DRIVE,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.FRONT_LEFT_DRIVE_REVERSE,
            Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
            Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
        new CCSparkMax(
            "Front Left Turn",
            "flt",
            Constants.MotorConstants.FRONT_LEFT_TURN,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.FRONT_LEFT_TURN_REVERSE,
            Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
            Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
        Constants.SwerveConstants.FRONT_LEFT_ABSOLUTE_ENCODER,
        Constants.SwerveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET,
        "Front Left");

public static final SwerveModule backRight =
    new SwerveModule(
        new CCSparkMax(
            "Back Right Drive",
            "brd",
            Constants.MotorConstants.BACK_RIGHT_DRIVE,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.BACK_RIGHT_DRIVE_REVERSE,
            Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
            Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
        new CCSparkMax(
            "Back Right Turn",
            "brt",
            Constants.MotorConstants.BACK_RIGHT_TURN,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.BACK_RIGHT_TURN_REVERSE,
            Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
            Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
        Constants.SwerveConstants.BACK_RIGHT_ABSOLUTE_ENCODER,
        Constants.SwerveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET,
        "Back Right");

public static final SwerveModule backLeft =
    new SwerveModule(
        new CCSparkMax(
            "Back Left Drive",
            "bld",
            Constants.MotorConstants.BACK_LEFT_DRIVE,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.BACK_LEFT_DRIVE_REVERSE,
            Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
            Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
        new CCSparkMax(
            "Back Left Turn",
            "blt",
            Constants.MotorConstants.BACK_LEFT_TURN,
            MotorType.kBrushless,
            IdleMode.kBrake,
            Constants.MotorConstants.BACK_LEFT_TURN_REVERSE,
            Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
            Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
        Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER,
        Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET,
        "Back Left");

        gyro  = new AHRS(SPI.Port.kMXP);
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.DRIVE_KINEMATICS, new Rotation2d(gyro.getAngle()), swerveModulePositions, Constants.PoseConstants.hi);
        chassisSpeeds = new ChassisSpeeds();

        chassisSpeedsXSPidController = new PIDController (0.7,0,0);
        chassisSpeedsYSPidController = new PIDController (0.7,0,0);
        chassisSpeedsThetaPidController = new PIDController(0.4,0,0);
        timer.start();
        currentTime = timer.getFGPATimestamp();
        swerveModulePositions = {};
    

    }

//     Vision vision = new Vision();
//     Timer timer = new Timer(); //ik i should start this in an init() method but too lazy
//     timer.start();

//     PIDController chassisXSPidController = new PIDController(0.7,0,0);
//     PIDController chassisYSPidController = new PIDController(0.7, 0, 0);
//     PIDController chassisThetaPidController = new PIDController(0.4,0,0);


//     double currentTime = timer.get(); 
//     public final SwerveModule frontRight =
//     new SwerveModule(
//         new CCSparkMax(
//             "Front Right Drive",
//             "frd",
//             Constants.MotorConstants.FRONT_RIGHT_DRIVE,
//             MotorType.kBrushless,
//             IdleMode.kBrake,
//             Constants.MotorConstants.FRONT_RIGHT_DRIVE_REVERSE,
//             Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
//             Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
//         new CCSparkMax(
//             "Front Right Turn",
//             "frt",
//             Constants.MotorConstants.FRONT_RIGHT_TURN,
//             MotorType.kBrushless,
//             IdleMode.kBrake,
//             Constants.MotorConstants.FRONT_RIGHT_TURN_REVERSE,
//             Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
//             Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
//         Constants.SwerveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER,
//         Constants.SwerveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET,
//         "Front Right");

// public static final SwerveModule frontLeft =
//     new SwerveModule(
//         new CCSparkMax(
//             "Front Left Drive",
//             "fld",
//             Constants.MotorConstants.FRONT_LEFT_DRIVE,
//             MotorType.kBrushless,
//             IdleMode.kBrake,
//             Constants.MotorConstants.FRONT_LEFT_DRIVE_REVERSE,
//             Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
//             Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
//         new CCSparkMax(
//             "Front Left Turn",
//             "flt",
//             Constants.MotorConstants.FRONT_LEFT_TURN,
//             MotorType.kBrushless,
//             IdleMode.kBrake,
//             Constants.MotorConstants.FRONT_LEFT_TURN_REVERSE,
//             Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
//             Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
//         Constants.SwerveConstants.FRONT_LEFT_ABSOLUTE_ENCODER,
//         Constants.SwerveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET,
//         "Front Left");

// public static final SwerveModule backRight =
//     new SwerveModule(
//         new CCSparkMax(
//             "Back Right Drive",
//             "brd",
//             Constants.MotorConstants.BACK_RIGHT_DRIVE,
//             MotorType.kBrushless,
//             IdleMode.kBrake,
//             Constants.MotorConstants.BACK_RIGHT_DRIVE_REVERSE,
//             Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
//             Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
//         new CCSparkMax(
//             "Back Right Turn",
//             "brt",
//             Constants.MotorConstants.BACK_RIGHT_TURN,
//             MotorType.kBrushless,
//             IdleMode.kBrake,
//             Constants.MotorConstants.BACK_RIGHT_TURN_REVERSE,
//             Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
//             Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
//         Constants.SwerveConstants.BACK_RIGHT_ABSOLUTE_ENCODER,
//         Constants.SwerveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET,
//         "Back Right");

// public static final SwerveModule backLeft =
//     new SwerveModule(
//         new CCSparkMax(
//             "Back Left Drive",
//             "bld",
//             Constants.MotorConstants.BACK_LEFT_DRIVE,
//             MotorType.kBrushless,
//             IdleMode.kBrake,
//             Constants.MotorConstants.BACK_LEFT_DRIVE_REVERSE,
//             Constants.ConversionConstants.HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION,
//             Constants.ConversionConstants.DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR),
//         new CCSparkMax(
//             "Back Left Turn",
//             "blt",
//             Constants.MotorConstants.BACK_LEFT_TURN,
//             MotorType.kBrushless,
//             IdleMode.kBrake,
//             Constants.MotorConstants.BACK_LEFT_TURN_REVERSE,
//             Constants.ConversionConstants.TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS,
//             Constants.ConversionConstants.TURN_MOTOR_RADIANS_PER_SECOND),
//         Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER,
//         Constants.SwerveConstants.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET,
//         "Back Left");

//     SwerveModule [] swerveModules = {frontLeft, frontRight, backLeft, backRight};
//     SwerveModuleState[] swerveModuleStates = {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
//     SwerveModulePosition[] swerveModulePositions;
//     AHRS gyro = new AHRS(SPI.Port.kMXP);
//     SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.DRIVE_KINEMATICS, new Rotation2d(gyro.getAngle()), swerveModulePositions, Constants.PoseConstants.hi);
//     ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    

    public void setChassisSpeeds(double xSpeed, double ySpeed, double turnSpeed){
        chassisSpeeds.vxMetersPerSecond = xSpeed;
        chassisSpeeds.vyMetersPerSecond = ySpeed;
        chassisSpeeds.omegaRadiansPerSecond = turnSpeed;
        SwerveModuleState[] desiredStates = Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(desiredStates);
    }

    

    public void setSwerveModulePositions(){
        swerveModulePositions[0] = new SwerveModulePosition(0, new Rotation2d(frontLeft.getAbsoluteEncoderWithOffsetRotations()));
        swerveModulePositions[1] = new SwerveModulePosition(0, new Rotation2d(frontRight.getAbsoluteEncoderWithOffsetRotations()));
        swerveModulePositions[2] = new SwerveModulePosition(0, new Rotation2d(backLeft.getAbsoluteEncoderWithOffsetRotations()));
        swerveModulePositions[3] = new SwerveModulePosition(0, new Rotation2d(backRight.getAbsoluteEncoderWithOffsetRotations()));
    }

    

    public void updateSwerveModulePositions(){
        swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getAbsoluteEncoderWithOffsetRotations()));
        swerveModulePositions[1] = new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getAbsoluteEncoderWithOffsetRotations()));
        swerveModulePositions[2] = new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getAbsoluteEncoderWithOffsetRotations()));
        swerveModulePositions[3] = new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getAbsoluteEncoderWithOffsetRotations()));
    }
    public void resetSwerveDrivePoseEstimator(){
        swerveDrivePoseEstimator.resetPosition(new Rotation2d(gyro.getAngle()), swerveModulePositions, new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getX(), swerveDrivePoseEstimator.getEstimatedPosition().getY(), gyro.getAngle()));
    }

    public void setModuleStates(SwerveModuleState[] desiredState){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, Constants.SwerveConstants.DRIVE_RATE_LIMIT);
        frontLeft.setState(desiredState[0]);
        frontRight.setState(desiredState[1]);
        backLeft.setState(desiredState[2]);
        backRight.setState(desiredState[3]);
    }

    public void updatePose(){
        swerveDrivePoseEstimator.addVisionMeasurement(vision.getPhotonPoseEstimator().getEstimatedGlobalPose(), currentTime, vision.getVisionStdDevs());
        swerveDrivePoseEstimator.updateWithTime(currentTime, gyro.getAngle(), swerveModulePositions);
    }

    public static SwerveDrivePoseEstimator getSwerveDrivePoseEstimator(){
        return swerveDrivePoseEstimator;
    }

    public static SwerveDrive currentInstance;
    public static SwerveDrive getInstance(){
        if (currentInstance == null){
            currentInstance = new SwerveDrive();
        }
        return currentInstance;
    }

    public double offsetToBestTagYaw(){
        
        var results = slimelight.getLatestResult();
        PhotonTrackedTarget tag  = results.getBestTarget();

        return tag.getYaw() - Units.rotationsToRadians(swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians());
        // return tag.getYaw() - photonPoseEstimator.getEstimatedGlobalPosition().getYaw();
    }

    public void alignToTagChassisSpeeds(double targetYaw){
        chassisThetaPidController.setSetpoint(targetYaw);
        while (chassisThetaPidController.atSetpoint())
        chassisSpeeds.omegaRadiansPerSecond =  chassisThetaPidController.calculate(swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians(), targetYaw);
        


    }

        private ChassisSpeeds angularPIDCalc(
            Supplier<Rotation2d> desiredRotation) {
        double pid = angularDrivePID.calculate(getAdjustedYaw(gyro.getAngle().getRadians()).getDegrees(), desiredRotation.get().getDegrees());

        ChassisSpeeds speeds = new ChassisSpeeds(swerveDrivePoseEstimator.getEstimatedPosition().getX(), swerveDrivePoseEstimator.getEstimatedPosition().getY(),
                MathUtil.clamp(
                        chassisThetaPidController.atSetpoint() ? 0 : pid + (Constants.SwerveConstants.angularDriveKS * Math.signum(pid)),
                        -SwerveConstants.TURN_RATE_LIMIT, SwerveConstants.TURN_RATE_LIMIT));

        return speeds;
    }

    public static double getAdjustedYaw(double angle){
        while (angle > Math.PI){
            angle -= 2*Math.PI;

        }
        while (angle < -Math.PI){
            angle += 2*Math.PI;
        }
        return angle;
    }

    public boolean atPoseSetpoint(){}
    
    public void pidToPose(Pose2d desiredPose){
        double xSpeed = chassisXSPidController(swerveDrivePoseEstimator.getEstimatedPosition().getX(), desiredPose.getX());
        double ySpeed = chassisYSPidController(swerveDrivePoseEstimator.getEstimatedPosition().getY(), desiredPose.getY());

        

    }

    public Command updateChassisSpeedsCommand(double xSpeed, double ySpeed, double thetaSpeed){
        
    }
    

}
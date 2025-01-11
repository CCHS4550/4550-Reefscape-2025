package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.maps.Constants;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase{
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    public  PhotonCamera slimelight = new PhotonCamera ("slimelight");
    slimelight.setDriverMode(true);
    slimelight.setPipelineIndex(Constants.cameraOne.CAMERA_ONE_PIPELINE);
    double pipelineLatency;
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator (aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, slimelight, Constants.cameraOne.ROBOT_TO_CAM);
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    

    public PhotonPoseEstimator getPhotonPoseEstimator(){
        return photonPoseEstimator;
    }

    public static double getEstimatedPose(){
        return photonPoseEstimator.getEstimatedGlobalPose();
    }

    public static double getEstimatedPoseX(){
        return photonPoseEstimator.getEstimatedGlobalPose().getX();
    }

    public static double getEstimatedPoseY(){
        return photonPoseEstimator.getEstimatedGlobalPose().getY();
    }

    public static double getEstimatedPoseYaw(){
        return photonPoseEstimator.getEstimatedGlobalPose().getRadians();
    }

    
    Matrix<N3, N1> visionStdDevs ; //idk how tf i declare this

    public Matrix<N3, N1> getVisionStdDevs(){
        return visionStdDevs;
    }

    public double alignToSpeaker(boolean alliance){
    //    double currentRotationRadians= SwerveDrive.getAdjustedYaw(SwerveDrive.getSwerveDrivePoseEstimator().getEstimatedPosition().getRotation().getRadians());
       double xTransform;
       double yTransform;
       double alignAngle;
       // assume that you are facing the april tag, new fix in the works
       if (alliance){
        xTransform = Constants.AprilTags.aprilTagPoses[4].getX() - SwerveDrive.getSwerveDrivePoseEstimator().getEstimatedPosition().getX();
        yTransform = Constants.AprilTags.aprilTagPoses[4].getY() - SwerveDrive.getSwerveDrivePoseEstimator().getEstimatedPosition().getY();
        alignAngle = Math.atan2(xTransform, yTransform)+180;
       }
       else {
        xTransform = Constants.AprilTags.aprilTagPoses[7].getX() - SwerveDrive.getSwerveDrivePoseEstimator().getEstimatedPosition().getX();
        yTransform = Constants.AprilTags.aprilTagPoses[7].getY() - SwerveDrive.getSwerveDrivePoseEstimator().getEstimatedPosition().getY();
        alignAngle = Math.atan2(xTransform, yTransform);
       }
    
       

       
    }
    

}
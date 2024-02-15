package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;


public class PhotonVisionSubsystem extends SubsystemBase{

    private final PhotonCamera camera_1;
    private PhotonPipelineResult camera1Data;
    // private boolean camera1HasTargets;
    // private List<PhotonTrackedTarget> camera1TargetsList;
    // private PhotonTrackedTarget bestCamera1Target;
    // private Transform2d camera1Pose;
    // private List<TargetCorner> camera1TargetCorners;
    private int counter = 0;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    

    public PhotonVisionSubsystem(){
        camera_1 = new PhotonCamera("Global Shutter Camera");
        camera1Data = getCamera1Data();
        


        
        //var camera1Data = camera_1.getLatestResult();
        
    }

    public PhotonPipelineResult getCamera1Data() {
        var camera1Data = camera_1.getLatestResult();
        //camera1HasTargets = camera1Data.hasTargets();
        //camera1TargetsList = camera1Data.getTargets();
        //bestCamera1Target = camera1Data.getBestTarget();
        
        return camera1Data;
    }

    public List getCameraTargets(PhotonPipelineResult cameraData) {
        List<PhotonTrackedTarget> cameraTargetsList = cameraData.getTargets();
        return cameraTargetsList;
    }

    public boolean doesCameraHaveTargets(PhotonPipelineResult cameraData) {
        boolean cameraHasTargets = cameraData.hasTargets();
        return cameraHasTargets;
    }

    public PhotonTrackedTarget getBestCameraTarget(PhotonPipelineResult cameraData) {
        PhotonTrackedTarget bestCameraTarget = cameraData.getBestTarget();
        return bestCameraTarget;
    }

    public double getTargetPitch(PhotonTrackedTarget target) {
        double pitch = target.getPitch();
        return pitch;
    }

        public double getTargetSkew(PhotonTrackedTarget target) {
        double skew = target.getSkew();
        return skew;
    }
    

    public double getTargetYaw(PhotonTrackedTarget target) {
        double yaw = target.getYaw();
        return yaw;
    }

    public double getTargetArea(PhotonTrackedTarget target) {
        double area = target.getArea();
        return area;
    }

    // public Transform2d getCameraTargetPose(PhotonTrackedTarget target) {
    //     Transform2d pose = target.getCameraToTarget;
    // }


    // public void getCamera1Data(){
    //     var camer1Data = camera_1.getLatestResult();
    //     camera1HasTargets = camera1Data.hasTargets();

    //     camera1TargetsList = camera1Data.getTargets();
    //     bestCamera1Target = camera1Data.getBestTarget();
    // }

    @Override
    public void periodic() {
        camera1Data = getCamera1Data();
        // if (counter != 20) {
        //     counter +=1;
        // }
        // else if (counter == 20){
        //     this.camera1Data = getCamera1Data();
        // }
        //PhotonPipelineResult camera1Data = getCamera1Data();
        SmartDashboard.putBoolean("Does Camera1 Have Targets", this.doesCameraHaveTargets(camera1Data));
        //SmartDashboard.putList("Camera1 Targets List", this.getCameraTargets(camera1Data));
        PhotonTrackedTarget cameraBestTarget = getBestCameraTarget(camera1Data);
        if (cameraBestTarget != null) {
        SmartDashboard.putNumber("Best Pose Yaw", this.getTargetYaw(cameraBestTarget));
        SmartDashboard.putNumber("Best Pose Skew", this.getTargetSkew(cameraBestTarget));
        SmartDashboard.putNumber("Best Pose Pitch", this.getTargetPitch(cameraBestTarget));
        SmartDashboard.putNumber("Best Pose Area", this.getTargetArea(cameraBestTarget));
        }

    }
    
}

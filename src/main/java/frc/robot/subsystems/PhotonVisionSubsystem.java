package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;

public class PhotonVisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera_1;
    private PhotonPipelineResult camera1Data;


    private boolean isDataValid;
    private double DistanceToTarget;
    private double FilteredDistanceToTarget;
    private final MedianFilter m_distanceFilter = new MedianFilter(5);
   
    private final double CAMERA_HEIGHT_METERES = Units.inchesToMeters(10);
    private final double TARGET_HEIGHT_METERS = Units.inchesToMeters(50);
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    AprilTagFieldLayout AprilTagFieldLayout1 = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final PhotonPoseEstimator odometry;
    private double EstimatedPoseNorm;
    private double FilteredEsimatedPoseNorm;
    private double FilteredDistanceToTargetOnField;



    public  PhotonVisionSubsystem(Transform3d pose) {
        camera_1 = new PhotonCamera("Global_Shutter_Camera");
        camera1Data = getCamera1Data();

        odometry = new PhotonPoseEstimator(AprilTagFieldLayout1, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera_1, pose);

        odometry.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }
    public boolean doesCameraHaveTargets(PhotonPipelineResult cameraData) {
        boolean cameraHasTargets = cameraData.hasTargets();
        return cameraHasTargets;
    }
    public List <PhotonTrackedTarget> getCameraTargets(PhotonPipelineResult cameraData) {
        List<PhotonTrackedTarget> cameraTargetsList = cameraData.getTargets();
        return cameraTargetsList;
    }
    public PhotonTrackedTarget getBestCamera1Target(PhotonPipelineResult cameraData) {
        PhotonTrackedTarget bestCameraTarget = cameraData.getBestTarget();
        return bestCameraTarget;
    }


    
    public PhotonPipelineResult getCamera1Data() {
        var camera1Data = camera_1.getLatestResult();
        return camera1Data;
    }
    public double getTargetYaw (PhotonTrackedTarget target) {
        double yaw = target.getYaw();
        return yaw;
    }
    public double getTargetPitch (PhotonTrackedTarget target) {
        double pitch = target.getPitch();
        return pitch;
    }
    public double getTargetSkew (PhotonTrackedTarget target) {
        double skew = target.getSkew();
        return skew;
    }
    public double getTargetArea (PhotonTrackedTarget target) {
        double area = target.getArea();
        return area;
    }
    public double getRangeToTarget(PhotonTrackedTarget target) {
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            TARGET_HEIGHT_METERS,
             CAMERA_PITCH_RADIANS,
            CAMERA_HEIGHT_METERES,
            Units.degreesToRadians(getTargetPitch(target))
          );
          return range;
    }
    
    public double getDistanceToTarget() {
        return DistanceToTarget;
    }

    public double getFilteredDistance(){
        return FilteredDistanceToTarget;
    }

    public boolean isDataValid() {
        return isDataValid;
    }

    @Override
    public void periodic() {

    camera1Data = getCamera1Data();
    isDataValid = false;
    DistanceToTarget = -1;
    //if ( ) {
        Optional <EstimatedRobotPose> currentPose =
        checkValidResults(camera1Data.targets)? odometry.update(camera1Data) : Optional.empty();
   // }
    
      SmartDashboard.putBoolean("Does cam 1 have targets?", this.doesCameraHaveTargets(camera1Data));
     // PhotonTrackedTarget cameraBestTarget = getBestCamera1Target(camera1Data);
      if (currentPose.isPresent()) {
                    

        /* 
        if (cameraBestTarget.getFiducialId() == 7 || cameraBestTarget.getFiducialId() == 4) {

             EstimatedPoseNorm = currentPose.get().estimatedPose.getTranslation().getNorm();
            EstimatedPoseNorm = Units.metersToInches(EstimatedPoseNorm);
            FilteredEsimatedPoseNorm = m_distanceFilter.calculate(EstimatedPoseNorm);
            

            DistanceToTarget = cameraBestTarget.getBestCameraToTarget().getTranslation().getNorm();
            DistanceToTarget = Units.metersToInches(DistanceToTarget);
            FilteredDistanceToTarget = m_distanceFilter.calculate(DistanceToTarget);

            FilteredDistanceToTargetOnField = FilteredEsimatedPoseNorm - FilteredDistanceToTarget;  

       }
        
            if ((DistanceToTarget > 0 && DistanceToTarget < 216.5) &&  
                (cameraBestTarget.getFiducialId() == 7 || cameraBestTarget.getFiducialId() == 4)) {
                isDataValid = true;
            }
            */
                Pose3d Tag7Pose = AprilTagFieldLayout1.getTagPose(7).get();
                EstimatedPoseNorm = currentPose.get().estimatedPose.getTranslation().minus(Tag7Pose.getTranslation()).getNorm();
                EstimatedPoseNorm = Units.metersToInches(EstimatedPoseNorm);
                FilteredEsimatedPoseNorm = m_distanceFilter.calculate(EstimatedPoseNorm);
                isDataValid = true;
      }
            SmartDashboard.putNumber("Distance to target", getDistanceToTarget());
            SmartDashboard.putNumber("Filtered Distance", FilteredDistanceToTarget);
            SmartDashboard.putNumber("Filtered Pose Dist", FilteredEsimatedPoseNorm);
            SmartDashboard.putNumber("Calculated distance", FilteredDistanceToTargetOnField);       
            SmartDashboard.putBoolean("is data valid?", isDataValid());


    }

    private boolean checkValidResults(List <PhotonTrackedTarget> result) {
        for (PhotonTrackedTarget target: result) {
            if (target.getFiducialId() ==  7 || target.getFiducialId() == 4 ) {
                return true;
            
            }
        }
        return false;
    }




}

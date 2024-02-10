package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.targeting.proto.PhotonPipelineResultProto;

public class PhotonVisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera_1;
    private PhotonPipelineResult camera1Data;


    private boolean isDataValid;
    private double DistanceToTarget;
   
    private int counter = 0;
    private final double CAMERA_HEIGHT_METERES = Units.inchesToMeters(10);
    private final double TARGET_HEIGHT_METERS = Units.inchesToMeters(50);
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    AprilTagFieldLayout AprilTagFieldLayout1 = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    Transform3d RobotToCam = new Transform3d(new Translation3d(0.5,0.0, 0.5), new Rotation3d(0,0,0));
    PhotonPoseEstimator photonPoseEstimator;

    private  Optional<Pose3d> TagPose;
    private Pose2d Tagpose2d;
    private  Translation3d m_translation;
    private  Rotation3d m_rotation;
    //PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout1, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera_1, RobotToCam);


    public  PhotonVisionSubsystem() {
        camera_1 = new PhotonCamera("Global_Shutter_Camera");
        camera1Data = getCamera1Data();
    photonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout1, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera_1, RobotToCam);
        // var camera_1Data = camera_1.getLatestResult();
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

    public boolean isDataValid() {
        return isDataValid;
    }

    @Override
    public void periodic() {

    camera1Data = getCamera1Data();
    isDataValid = false;
    DistanceToTarget = -1;

      SmartDashboard.putBoolean("Does cam 1 have targets?", this.doesCameraHaveTargets(camera1Data));
      PhotonTrackedTarget cameraBestTarget = getBestCamera1Target(camera1Data);
      if (cameraBestTarget != null) {

        SmartDashboard.putNumber("Best Pose Yaw", this.getTargetYaw(cameraBestTarget));
        SmartDashboard.putNumber("Best Target Skew", getTargetSkew(cameraBestTarget));
        SmartDashboard.putNumber("Best Target Pitch", getTargetPitch(cameraBestTarget));
        SmartDashboard.putNumber("Best Target Area", getTargetArea(cameraBestTarget));
        SmartDashboard.putNumber("Range to target (meters)", getRangeToTarget(cameraBestTarget));
        if (cameraBestTarget.getFiducialId() == 7 || cameraBestTarget.getFiducialId() == 4) {


            DistanceToTarget = cameraBestTarget.getBestCameraToTarget().getTranslation().getNorm();
            

        }
        
            if ((DistanceToTarget > 0 && DistanceToTarget < 5.5) &&  
                (cameraBestTarget.getFiducialId() == 7 || cameraBestTarget.getFiducialId() == 4)) {
                isDataValid = true;
            }

        //photonPoseEstimator.update();
        //TagPose = photonPoseEstimator.getFieldTags().getTagPose(cameraBestTarget.getFiducialId());
        
        //m_translation = TagPose.get().getTranslation();
        //m_rotation = TagPose.get().getRotation();
        //Tagpose2d = new Pose2d( m_translation.toTranslation2d(), m_rotation.toRotation2d());

    
        //SmartDashboard.putNumber("x to target pose", TagPose.get().getX());
        //SmartDashboard.putNumber("2d x to target", Tagpose2d.getX());
      }
            SmartDashboard.putNumber("Distance to target", getDistanceToTarget());
            SmartDashboard.putBoolean("is data valid?", isDataValid());


    }






}

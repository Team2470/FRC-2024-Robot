package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class AlignYawWithTAG extends SequentialCommandGroup {

    public AlignYawWithTAG(Drivetrain drive, PhotonVisionSubsystem m_camera1) {
        addCommands(
            // Commands.runOnce(()->m_angle = null),
            // Commands.run(()->{
            //     int tag = (int)LimelightHelpers.getFiducialID(kLimelight);
            //     if (!(tag == 11 || tag == 12 || tag == 13 || tag == 14 || tag == 15 || tag == 16)) {
            //         // Not a target
            //         return;
            //     }

            //     // get the target pose from the field info.
            //     Optional<Pose3d> tagPose =  kField.getTagPose(tag);
            //     if (tagPose.isPresent()) {
            //         m_angle = tagPose.get().getRotation().toRotation2d().getDegrees();
            //     }

            //     // if (tag == 11) {
            //     // } else if (tag == 12) {
            //     //     m_angle = 0.0;
            //     // } else if (tag == 13) {
            //     //     m_angle = 0.0;
            //     // } else if (tag == 14) {
            //     //     m_angle = 0.0;
            //     // } else if (tag == 15) {
            //     //     m_angle = 0.0;
            //     // } else  if (tag == 16) {
            //     //     m_angle = 0.0;
            //     // }
            // }).until(()->m_angle != null),
            new ParallelCommandGroup(
                new DriveWithController(
                    drive,
                    // X Move Velocity - Forward
                    ()-> 0.0,

                    // Y Move Velocity - Strafe
                    ()-> 0.0,

                    // Rotate Angular velocity
                    () -> 0.0,

                    // Field Orientated
                    () -> false,

                    // Slow Mode
                    () -> false,		
                
                    // Disable X Movement
                    () -> false,

                    // Disable Y Movement
                    () -> false,

                    // Vision Heading Override
                    // ()-> null,
                    () -> {
                            if (m_camera1.doesCameraHaveTarget())
                                return m_camera1.getRobotYaw();
                                return null;
                    },

                    // Field oriented override
                    () -> null,
                    false
                )
            )
        );
    }    
}

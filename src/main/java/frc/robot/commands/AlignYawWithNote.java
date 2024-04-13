package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;

public class AlignYawWithNote extends SequentialCommandGroup {

    private final static String kLimelight = "limelight-shooter";
    private final static AprilTagFieldLayout kField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final PIDController m_txPID = new PIDController(0.025, 0, 0);
    private final PIDController m_tyPID = new PIDController(0.025,0,0);


    private Double m_angle = 0.0;

    public AlignYawWithNote(Drivetrain drive) {
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
            Commands.runOnce(()->{
                m_txPID.reset();
                m_txPID.setTolerance(1);

                m_tyPID.reset();
                m_tyPID.setTolerance(1);
            }),
            new ParallelCommandGroup(
                new DriveWithController(
                    drive,
                    // X Move Velocity - Forward
                    ()-> -0.4,

                    // Y Move Velocity - Strafe
                    ()-> 0,

                    // Rotate Angular velocity
                    () -> MathUtil.clamp(m_txPID.calculate(LimelightHelpers.getTX(kLimelight), 0), -1, 1),

                    // Field Orientated
                    () -> false,

                    // Slow Mode
                    () -> false,		
                
                    // Disable X Movement
                    () -> false,

                    // Disable Y Movement
                    () -> false,

                    // Vision Heading Override
                    () -> null,

                    // Field oriented override
                    () -> null,
                    false
                ),
                Commands.run(()->{
                    SmartDashboard.putNumber("TRAP tx error", m_txPID.getPositionError());
                    SmartDashboard.putNumber("TRAP ty error", m_tyPID.getPositionError());
                    SmartDashboard.putNumber("Tx LL", LimelightHelpers.getTX(kLimelight));
                })
            ).until(()->m_txPID.atSetpoint() && m_tyPID.atSetpoint()  && LimelightHelpers.getTV(kLimelight))
        );
    }    
}

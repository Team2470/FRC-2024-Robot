package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;

public class ALignTrapShootCommand extends SequentialCommandGroup {

    private final static String kLimelight = "limelight-shooter";
    private final static AprilTagFieldLayout kField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final PIDController m_txPID = new PIDController(0.025, 0, 0);
    private final PIDController m_tyPID = new PIDController(0.025,0,0);

    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
			DriveConstants.kDriveKinematics,
			new Rotation2d(),
			new SwerveModulePosition()[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
			},
			new Pose2d(),
			VecBuilder.fill(0.1, 0.1, 0.1),
			VecBuilder.fill(1.5, 1.5, 1.5)
    ); // 7028 uses 1.5



    private Double m_angle = 0.0;

    public ALignTrapShootCommand(Drivetrain drive) {
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
                    ()-> MathUtil.clamp(m_tyPID.calculate(LimelightHelpers.getTY(kLimelight), 0), -0.2, 0.2),

                    // Y Move Velocity - Strafe
                    ()-> MathUtil.clamp(m_txPID.calculate(LimelightHelpers.getTX(kLimelight), 0), -0.2, 0.2),

                    // Rotate Angular velocity
                    () ->  0.0,

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
                    () -> m_angle,
                    false
                ),
                Commands.run(()->{
                    m_odometry.update(drive.getIMUHeading(), drive.getModulePositions());
                    var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

                    Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

                    if (lastResult.valid) {
                        m_odometry.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
                    }

                    SmartDashboard.putNumber("TRAP tx error", m_txPID.getPositionError());
                    SmartDashboard.putNumber("TRAP ty error", m_tyPID.getPositionError());
                })
            ).until(()->m_txPID.atSetpoint() && m_tyPID.atSetpoint()  && LimelightHelpers.getTV(kLimelight))
        );
    }    
}

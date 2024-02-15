// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.SimpleFlywheel;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final PhotonVisionSubsystem m_camera1 = new PhotonVisionSubsystem(VisionConstants.kFrontRightCamera);
  private final SimpleFlywheel m_simpleFlywheelLeft = new SimpleFlywheel(FlyWheelConstants.kLeftID, true);
  private final SimpleFlywheel m_simpleFlywheelRight = new SimpleFlywheel(FlyWheelConstants.kRightID, false);
  private final ShooterPivot m_ShooterPivot = new ShooterPivot();
  private final Constants m_Constants = new Constants();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final CommandXboxController m_sysIDcontroller  = new CommandXboxController(1);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
        

    SmartDashboard.putNumber("Select Left Voltage", 0);
    SmartDashboard.putNumber("Select Right Voltage", 0);
    SmartDashboard.putNumber("Select Left RPM", 100);
    SmartDashboard.putNumber("Select Right RPM", 0);
    SmartDashboard.putNumber("kP", FlyWheelConstants.kP);
    SmartDashboard.putNumber("kI", FlyWheelConstants.kI);
    SmartDashboard.putNumber("kD", FlyWheelConstants.kD);
    SmartDashboard.putNumber("kF", FlyWheelConstants.kF);
    SmartDashboard.putNumber("Select Shooter Pivot Angle", 0);
    SmartDashboard.putNumber("Select Distance", 0);

  
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_controller.a().whileTrue(m_simpleFlywheel.spinCommand(2));
    // m_controller.b().whileTrue(m_simpleFlywheel.spinCommand(4));
    // m_controller.x().whileTrue(m_simpleFlywheel.spinCommand(6));
    // m_controller.y().whileTrue(m_simpleFlywheel.spinCommand(8));
    //m_controller.rightBumper().whileTrue(m_simpleFlywheel.spinCommand(-2));
    m_controller.rightTrigger().whileTrue(m_simpleFlywheelLeft.openLoopCommand(()-> SmartDashboard.getNumber("Select Left Voltage", 0)));
    m_controller.rightTrigger().whileTrue(m_simpleFlywheelRight.openLoopCommand(()-> SmartDashboard.getNumber("Select Right Voltage", 0)));
    m_controller.leftTrigger().whileTrue(m_simpleFlywheelLeft.pidCommand(()-> SmartDashboard.getNumber("Select Left RPM", 0)));
    m_controller.leftTrigger().whileTrue(m_simpleFlywheelRight.pidCommand(()-> SmartDashboard.getNumber("Select Right RPM", 0)));
    m_controller.b().whileTrue(m_ShooterPivot.openLoopCommand(2));
    m_controller.a().whileTrue(m_ShooterPivot.openLoopCommand(-2));
    // m_controller.x().whileTrue(m_ShooterPivot.goToAngleCommand(37.08984375));
    m_controller.x().whileTrue(m_ShooterPivot.goToAngleCommand(()-> SmartDashboard.getNumber("Select Shooter Pivot Angle", 0)));
    m_controller.y().whileTrue(m_ShooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle(m_camera1.getFilteredDistance())));
    m_controller.y().whileTrue(m_simpleFlywheelLeft.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.getFilteredDistance())));
    m_controller.y().whileTrue(m_simpleFlywheelRight.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.getFilteredDistance())));
    //dont go 2800-3200 rpm (Harmonics ;] )
  }


  
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
public void teleopInit(){

}

  public void periodic() {
    SmartDashboard.putNumber("Angle", ShooterPivotConstants.getAngle(m_camera1.getDistanceToTarget()));
    SmartDashboard.putNumber("RPM", FlyWheelConstants.getRPM(m_camera1.getDistanceToTarget()));
  }

}




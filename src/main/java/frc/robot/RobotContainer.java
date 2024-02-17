// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.kennedyrobotics.auto.AutoSelector;
import com.kennedyrobotics.hardware.misc.RevDigit;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.SimpleFlywheel;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveWithController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.vision.VisionIOPhoton;
import frc.robot.subsystems.SimpleShooterFeeder;
import frc.robot.subsystems.TimeOfFlightSensorTest;
import frc.robot.subsystems.PhotonVisionSubsystem;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // OI
  private final CommandXboxController m_controller = new CommandXboxController(0);
	private final CommandJoystick m_buttonPad = new CommandJoystick(1);
  // The robot's subsystems and commands are defined here...
  private final PhotonVisionSubsystem m_camera1 = new PhotonVisionSubsystem(VisionConstants.kFrontRightCamera);
  private final SimpleFlywheel m_simpleFlywheelLeft = new SimpleFlywheel(FlyWheelConstants.kLeftID, true);
  private final SimpleFlywheel m_simpleFlywheelRight = new SimpleFlywheel(FlyWheelConstants.kRightID, false);
  private final ShooterPivot m_ShooterPivot = new ShooterPivot();
  private final Constants m_Constants = new Constants();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final SimpleShooterFeeder m_SimpleShooterFeeder = new SimpleShooterFeeder(20);
  private final TimeOfFlightSensorTest m_TOF1 = new TimeOfFlightSensorTest();


  // Auto
  private final RevDigit m_revDigit;
  private final AutoSelector m_autoSelector;




  

  public RobotContainer() {
    // CameraServer.startAutomaticCapture();

    m_simpleFlywheelLeft.setDefaultCommand(m_simpleFlywheelLeft.pidCommand(1000));
    m_simpleFlywheelRight.setDefaultCommand(m_simpleFlywheelRight.pidCommand(1000));
    m_ShooterPivot.setDefaultCommand(m_ShooterPivot.goToAngleCommand(45));

    m_revDigit = new RevDigit();
    m_revDigit.display("3081");
    m_autoSelector = new AutoSelector(m_revDigit, "DFLT", new SequentialCommandGroup(new PrintCommand("OOPS")));

    // Initialize other autos here
    // TODO
    m_autoSelector.registerCommand("Test", "TEST", m_drivetrain.createAutoPath(
      null, "New Auto", AutoConstants.kPathConstraints));

    m_autoSelector.initialize();

    
      
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
  // m_controller.x().whileTrue(m_simpleFlywheel.spinCommand(6));
    // m_controller.y().whileTrue(m_simpleFlywheel.spinCommand(8));
    //m_controller.rightBumper().whileTrue(m_simpleFlywheel.spinCommand(-2));
    // m_controller.rightTrigger().whileTrue(m_simpleFlywheelLeft.openLoopCommand(()-> SmartDashboard.getNumber("Select Left Voltage", 0)));
    // m_controller.rightTrigger().whileTrue(m_simpleFlywheelRight.openLoopCommand(()-> SmartDashboard.getNumber("Select Right Voltage", 0)));
    // m_buttonPad.button(10).whileTrue(m_simpleFlywheelLeft.pidCommand(()-> SmartDashboard.getNumber("Select Left RPM", 0)));
    // m_buttonPad.button(11).whileTrue(m_simpleFlywheelRight.pidCommand(()-> SmartDashboard.getNumber("Select Right RPM", 0)));
    m_buttonPad.button(8).whileTrue(m_ShooterPivot.openLoopCommand(2));
    m_buttonPad.button(12).whileTrue(m_ShooterPivot.openLoopCommand(-2));
    // m_controller.x().whileTrue(m_ShooterPivot.goToAngleCommand(37.08984375));
    // m_controller.x().whileTrue(m_ShooterPivot.goToAngleCommand(()-> SmartDashboard.getNumber("Select Shooter Pivot Angle", 0)));
    // m_controller.y().whileTrue(m_ShooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle(m_camera1.getFilteredDistance())));
    // m_controller.y().whileTrue(m_simpleFlywheelLeft.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.getFilteredDistance())));
    // m_controller.y().whileTrue(m_simpleFlywheelRight.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.getFilteredDistance())));
    // m_controller.rightBumper().whileTrue(m_ShooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle(m_camera1.getFilteredDistance())));

    // // m_controller.rightBumper().whileTrue(m_simpleFlywheel.spinCommand(-2));
    m_buttonPad.button(10).whileTrue(m_SimpleShooterFeeder.SimpleShooterFeeder_forwardsCommand());
    m_buttonPad.button(11).whileTrue(m_SimpleShooterFeeder.SimpleShooterFeeder_reverseCommand());
    // m_controller.b().whileTrue(m_SimpleShooterFeeder.SimpleShooterFeeder_reverseCommand());
    // //m_controller.x().whileTrue(m_TOF1.sequenceTest(m_SimpleShooterFeeder));
    // m_controller.y().whileTrue(m_TOF1.variableVoltageTest(m_SimpleShooterFeeder));

    // m_buttonPad.button(1).whileTrue(m_ShooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle(m_camera1.getFilteredDistance())));
    m_buttonPad.button(1).whileTrue(m_simpleFlywheelLeft.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())));
    m_buttonPad.button(1).whileTrue(m_simpleFlywheelRight.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())));
    m_buttonPad.button(1).whileTrue(m_ShooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle(m_camera1.FilteredEsimatedPoseNorm())));

    m_buttonPad.button(1).whileTrue(new ParallelCommandGroup(
      m_ShooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle((m_camera1.FilteredEsimatedPoseNorm()))),
      m_simpleFlywheelLeft.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
      m_simpleFlywheelRight.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
      m_simpleFlywheelLeft.feederShooterCommand(m_SimpleShooterFeeder)
    ));

    m_buttonPad.button(2).whileTrue(new ParallelCommandGroup(
          m_ShooterPivot.goToAngleCommand(56.92836363),
          m_simpleFlywheelLeft.pidCommand(2326.626089),
          m_simpleFlywheelRight.pidCommand(2326.626089)

    ));
//56.92836363
    m_buttonPad.button(3).whileTrue(new ParallelCommandGroup(
          m_ShooterPivot.goToAngleCommand(48.779296875),
          m_simpleFlywheelLeft.pidCommand(-1500),
          m_simpleFlywheelRight.pidCommand(-1500),
          m_TOF1.feederIntakeCommand(m_SimpleShooterFeeder)

    ));
    m_buttonPad.button(4).whileTrue(new ParallelCommandGroup(
        m_ShooterPivot.goToAngleCommand(44),
        m_simpleFlywheelLeft.pidCommand(1300),
        m_simpleFlywheelRight.pidCommand(1300)

    ));

    m_buttonPad.button(5).whileTrue(new ParallelCommandGroup(
        m_ShooterPivot.goToAngleCommand(57.91),
        m_simpleFlywheelLeft.pidCommand(2300),
        m_simpleFlywheelRight.pidCommand(2300)

    ));



    //dont go 2800-3200 rpm (Harmonics ;] )
    // Configure default commands

    // Configure default commands
    m_drivetrain.setDefaultCommand(
        new DriveWithController(
            m_drivetrain,
            // X Move Velocity - Forward
            () -> -m_controller.getHID().getLeftY(),

            // Y Move Velocity - Strafe
            () -> -m_controller.getHID().getLeftX(),

            // Rotate Angular velocity
            () -> {
              double leftTrigger = m_controller.getHID().getLeftTriggerAxis();
              double rightTrigger = m_controller.getHID().getRightTriggerAxis();

              if (leftTrigger < rightTrigger) {
                return -rightTrigger;
              } else {
                return leftTrigger;
              }
            },

            // Field Orientated
            () -> !m_controller.getHID().getAButton(),

            // Slow Mode
            () -> m_drivetrain.getSlowMode() || m_controller.getHID().getXButton(),

            // Disable X Movement
            () -> m_controller.getHID().getBButton(),

            // Disable Y Movement
            () -> false,

            // Heading Override
            () -> {
              switch (m_controller.getHID().getPOV()) {
                case 0: return 0.0;
                case 180: return 180.0;
                default: return null;
              }
            }));

    m_controller
        .start()
        .onTrue(new InstantCommand(m_drivetrain::resetHeading)); // TODO this should also do
    // something with odometry? As
    // it freaks out

    m_controller.rightStick().toggleOnTrue(m_drivetrain.xStop());

    // m_controller.povRight().whileTrue(new RobotTurnToAngle(m_drivetrain, 0));

    // m_controller.povLeft().whileTrue(new RobotTurnToAngle(m_drivetrain, 180));
  }


  
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An ExampleCommand will run in autonomous
    return m_autoSelector.selected();
  }

  public void autonomousInit() {
    m_drivetrain.resetHeading();
    m_drivetrain.setNominalVoltages(AutoConstants.kAutoVoltageCompensation);
  }

  public void teleopInit() {
    m_drivetrain.setSlowMode(false);
    m_drivetrain.setNominalVoltages(DriveConstants.kDriveVoltageCompensation);
  }

  public void robotPeriodic() {
    SmartDashboard.putNumber("Angle", ShooterPivotConstants.getAngle(m_camera1.FilteredEsimatedPoseNorm()));
    SmartDashboard.putNumber("RPM", FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm()));
  }

  public void disabledPeriodic() {
    m_drivetrain.resetSteerEncoders();
  }

  private Command createTestAuto() {
    return m_drivetrain.createAutoPath(null, "New Path", AutoConstants.kPathConstraints);
  }


}




// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.kennedyrobotics.auto.AutoSelector;
import com.kennedyrobotics.hardware.misc.RevDigit;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.ALignTrapShootCommand;
import frc.robot.commands.AlignYawWithNote;
import frc.robot.commands.AlignYawWithTAG;
import frc.robot.commands.DriveWithController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Orchestra6;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.SimpleFlywheel;
import frc.robot.subsystems.SimpleShooterFeeder;
import frc.robot.subsystems.TimeOfFlightSensorTest;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Climber;
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
		private final CommandXboxController m_controller2 = new CommandXboxController(2);
	// The robot's subsystems and commands are defined here...
	private final PhotonVisionSubsystem m_camera1 = new PhotonVisionSubsystem(VisionConstants.kFrontRightCamera);
	private final SimpleFlywheel m_simpleFlywheelBottom = new SimpleFlywheel(FlyWheelConstants.kLeftID, true);
	private final SimpleFlywheel m_simpleFlywheelTop = new SimpleFlywheel(FlyWheelConstants.kRightID, false);
	private final ShooterPivot m_shooterPivot = new ShooterPivot();
	private final Drivetrain m_drivetrain = new Drivetrain();
	private final SimpleShooterFeeder m_feeder = new SimpleShooterFeeder(20);
	private final TimeOfFlightSensorTest m_TOF1 = new TimeOfFlightSensorTest(1);
	private final TimeOfFlightSensorTest m_TOF2 = new TimeOfFlightSensorTest(2);
	private final IntakePivot m_intakePivot = new IntakePivot();
	private final Intake m_intake = new Intake();
	private final Orchestra6 m_Orchestra6 = new Orchestra6(11,12,13,14,26,27);
	private final LEDSubsystem m_LEDs = new LEDSubsystem(m_controller);
	private final Climber m_ClimberLeft = new Climber(ClimberConstants.kLeftMotorID,
														ClimberConstants.kLeftServoChannel,
														ClimberConstants.kLeftExtendChannel,
														ClimberConstants.kLeftRetractChannel,
														true);


	private final Climber m_ClimberRight = new Climber(ClimberConstants.kRightMotorID,
														ClimberConstants.kRightServoChannel,
														ClimberConstants.kRightExtendChannel,
														ClimberConstants.kRightRetractChannel,
														false);
	// Auto
	private final RevDigit m_revDigit;
	private final AutoSelector m_autoSelector;

	private boolean slowMode;

  private final DigitalInput m_brakeButton = new DigitalInput(3);
  

	public RobotContainer() {
		SmartDashboard.putString("roboRio Serial Number", RobotController.getSerialNumber());
		// CameraServer.startAutomaticCapture();


		// Auto Selector
		m_revDigit = new RevDigit().display("2470");

		m_autoSelector = new AutoSelector(
		m_revDigit, "DFLT", new PrintCommand("!!! Invalid Auto Selected !!!")
		);

		NamedCommands.registerCommands(new HashMap<String, Command>() {{
			put("speaker-shoot", speakerShoot());
			put("speaker-shoot2", speakerShoot2());
			put("auto-shoot", new ParallelDeadlineGroup(
				autoShoot(), m_intakePivot.stowCommand()
			));
			put("pickup", intakeCommand2().until(()-> m_TOF2.isTOF1WithinRange()));
			put("idle", idleAuto().until(()-> m_TOF2.isTOF1WithinRange()));
			put("trackWhilePickup", intakeCommandAuto().until(()-> m_TOF2.isTOF1WithinRange()));		
			put("deploy-intake", m_intakePivot.deploy().until(()-> m_intakePivot.getAngle() < 10));
			put("Intake-up", m_intakePivot.stowCommand().until(()-> (m_intakePivot.getAngle() > 80)));
			put("IntakeP1", intakingCommand());
			put("IntakeP2", intakeUpCommand());
			put("45Degrees", m_shooterPivot.goToAngleCommand(45).until(()-> m_TOF2.isTOF1WithinRange())) ;
			// put("32", m_shooterPivot.goToAngleCommand(45).until(()-> m_TOF2.isTOF1WithinRange()));
			put("idle2", idleAuto2(27.00));
			put("coast", new InstantCommand(m_drivetrain::disableBrakeMode));
			put("break", new InstantCommand(m_drivetrain::enableBrakeMode));
			put("AP", autoPickUp());
		}});

		registerAutos(new HashMap<String, String>() {{
			//: basic autos - 2 note score
			// put("2SRC", "2SRC");
			// put("2CEN", "2CEN");
			// put("2AMP", "2AMP");

			// //: only moves - nothing else
			// //put("MOVE", "MOVE");

			// //: single note autos - shoot only
			// put("1SRC", "1SRC");
			// // put("1CEN", "1CEN");
			// // put("1AMP", "1AMP");
			
			// //: extra autos - lots of notes
			// // put("FAR1", "FAR1");	
			// // put("FAR2", "FAR2");
			// put("4SRC", "4SRC");	

			// // put("3SRC", "3SRC");
			// // put("3CEN", "3CEN");
			// //put("3AMP", "3AMP");
			// // put("test", "test");
			// put("4CNA", "4CNA");
			// put("4CNS", "4CNS");
			// //put("1MMR", "1midMR");
			// put("4CEN", "4CEN");
			// put("FAR3", "FAR3");
			// // put("3CNS", "3CNS");
			// put("FARB", "FARB");
			put("ATST", "ATST");
			put("TST1", "TST1");
			put("FAR6", "FAR6");
			put("3SRC", "3SRC");
			put("4CN1", "4CN1");
		}});

		m_autoSelector.initialize();

		// TODO Uncomment after test on robot that angles make sense
		setupShooter();
		addValuesToDashboard();
		configureBindings();

		m_LEDs.setDefaultCommand(Commands.run(() -> {
		if (m_intake.isRingIntaked()) {
			m_LEDs.changeIntakeGreen();
		} else {
			m_LEDs.changeIntakeRed();
		}

		if (m_TOF1.isTOF1WithinRange()) {
			m_LEDs.changeTOF1Green();
		} else {
			m_LEDs.changeTOF1Red();
		}

		if(m_camera1.doesCameraHaveTarget()) {
			m_LEDs.changeVisionGreen();
		} else {
			m_LEDs.changeVisionRed();
		}

		if (m_simpleFlywheelTop.isErrorInRange()) {
			m_LEDs.changeShooterGreen();
		} else if(m_simpleFlywheelTop.isErrorBelow()){
			m_LEDs.changeShooterYellow();
		} else if(m_simpleFlywheelTop.isErrorAbove()){
			m_LEDs.changeShooterRed();
		}

		}, m_LEDs));
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
		m_controller.povRight().toggleOnTrue(new ParallelCommandGroup(
			// m_Orchestra6.playMusicCommand(),
			// m_Orchestra6v21.playMusiCommand(),
			// m_Orchestra6v22.playMusiCommand(),
			// m_Orchestra6v23.playMusiCommand(),
			// m_Orchestra6v24.playMusiCommand(),
			m_simpleFlywheelBottom.pidCommand(0),
			m_simpleFlywheelTop.pidCommand(0)
		));

		m_controller.y().whileTrue(this.extendClimber());
		m_controller.b().whileTrue(this.retractClimber());
		// m_buttonPad.button(8).whileTrue(m_shooterPivot.openLoopCommand(2));
		// m_buttonPad.button(12).whileTrue(m_shooterPivot.openLoopCommand(-2));
		m_buttonPad.button(6).whileTrue(m_intakePivot.deploy());
		m_buttonPad.button(7).whileTrue(m_intakePivot.stowCommand());

		//m_controller.povUp().onTrue(new InstantCommand(()-> m_camera1.offset+=1));
		//m_controller.povDown().onTrue(new InstantCommand(()-> m_camera1.offset -=1));
		//m_controller.povLeft().onTrue(new InstantCommand(()-> m_camera1.offset = 0));
		m_buttonPad.button(8).whileTrue(passNote());
		// m_buttonPad.button(12).whileTrue(intakeUpCommand());
		// m_buttonPad.button(12).whileTrue(intakeCommand2());
		m_controller2.leftTrigger().whileTrue(intakeCommand2());
		m_controller2.rightTrigger().whileTrue(visionShootAndXStop());
		m_controller2.povUp().whileTrue(m_intakePivot.stowCommand());
		m_controller2.povDown().whileTrue(m_intakePivot.deploy());
		m_controller2.leftBumper().whileTrue(m_feeder.forward());
		m_controller2.rightBumper().whileTrue(m_intake.test_reverseCommand());
		m_controller2.y().whileTrue(	new ParallelCommandGroup(
			m_shooterPivot.goToAngleCommand(50),//48.779296875),
			m_simpleFlywheelBottom.pidCommand(-1700),
			m_simpleFlywheelTop.pidCommand(-1700),
			m_TOF1.feederIntakeCommand(m_feeder))
		);
		m_controller2.b().whileTrue(ampShoot());
		

		

		// m_buttonPad.button(11).whileTrue(new ParallelCommandGroup(
		//   m_ShooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle((m_camera1.FilteredEsimatedPoseNorm()))),
		//   m_simpleFlywheelBottom.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
		//   m_simpleFlywheelTop.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
		//   m_SimpleShooterFeeder.forward()
		// ));
		m_buttonPad.button(11).whileTrue(feed());
		m_buttonPad.button(10).whileTrue(m_intake.test_reverseCommand());

		// m_buttonPad.button(1).whileTrue(new ParallelCommandGroup(
		//   m_ShooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle((m_camera1.FilteredEsimatedPoseNorm()))),
		//   m_simpleFlywheelLeft.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
		//   m_simpleFlywheelRight.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
		//   m_simpleFlywheelLeft.feederShooterCommand(m_SimpleShooterFeeder)
		// ));
		m_buttonPad.button(1).whileTrue(visionShootAndXStop());
   		// m_buttonPad.button(1).whileTrue(m_shooterPivot.goToAngleCommand(()-> SmartDashboard.getNumber("Select Shooter Pivot Angle", 0))); 
		m_buttonPad.button(9).whileTrue(intakeCommand2());

		//keep
		// m_buttonPad.button(8).whileTrue(StageShoot());

		m_controller.back().whileTrue(new ParallelCommandGroup(
			m_simpleFlywheelBottom.pidCommand(10000),
			m_simpleFlywheelTop.pidCommand(10000)
		));

		//     m_buttonPad.button(6).whileTrue(new ParallelCommandGroup(
		//   m_ShooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle((m_camera1.FilteredEsimatedPoseNorm()))),
		//   m_simpleFlywheelLeft.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
		//   m_simpleFlywheelRight.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm()))
		// ));

		//     m_buttonPad.button(7).whileTrue(new ParallelCommandGroup(
		//       m_ShooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle(SmartDashboard.getNumber("Select Distance", 0))),
		//       m_simpleFlywheelLeft.pidCommand(()-> FlyWheelConstants.getRPM(SmartDashboard.getNumber("Select Distance", 0))),
		//       m_simpleFlywheelRight.pidCommand(()-> FlyWheelConstants.getRPM(SmartDashboard.getNumber("Select Distance", 0)))
		// ));

		// m_buttonPad.button(2).whileTrue(new ParallelCommandGroup(
		//       m_ShooterPivot.goToAngleCommand(59.92836363),
		//       m_simpleFlywheelBottom.pidCommand(2326.626089),
		//       m_simpleFlywheelTop.pidCommand(2326.626089)

		// ));

		m_buttonPad.button(2).whileTrue(speakerShoot());
	//56.92836363

		m_buttonPad.button(3).whileTrue(new ParallelCommandGroup(
			m_shooterPivot.goToAngleCommand(50),//48.779296875),
			m_simpleFlywheelBottom.pidCommand(-1700),
			m_simpleFlywheelTop.pidCommand(-1700),
			m_TOF1.feederIntakeCommand(m_feeder)
		));

		// m_buttonPad.button(4).whileTrue(new ParallelCommandGroup(
		//     m_ShooterPivot.goToAngleCommand(50),
		//     m_simpleFlywheelBottom.pidCommand(1250),
		//     m_simpleFlywheelTop.pidCommand(700)

		// ));

		m_buttonPad.button(4).whileTrue(ampShoot());
		m_buttonPad.button(12).whileTrue(ampShoot2());

		m_buttonPad.button(5).whileTrue(new ParallelCommandGroup(
			// m_shooterPivot.goToAngleCommand(()-> SmartDashboard.getNumber("Select Shooter Pivot Angle", 45.00)),
			m_shooterPivot.goToAngleCommand(57.91),
			m_simpleFlywheelBottom.pidCommand(2300),
			m_simpleFlywheelTop.pidCommand(2300)
		));
		// m_controller.rightBumper().whileTrue(new SequentialCommandGroup(
		// 	new ParallelDeadlineGroup(
		// 		new ALignTrapShootCommand(m_drivetrain),
		// 		m_shooterPivot.goToAngleCommand(45)
		// 		// m_simpleFlywheelBottom.pidCommand(2300),
		// 		// m_simpleFlywheelTop.pidCommand(2300)
		// 	)
		// ));

		// m_buttonPad.button(9).whileTrue(new SequentialCommandGroup(
		// 	new ParallelDeadlineGroup(
		// 		intakeCommand2(),
		// 		new SequentialCommandGroup(
		// 			new WaitUntilCommand(()-> m_intakePivot.getAngle() < 1),
		// 			new AlignYawWithNoteController(m_drivetrain, m_controller)
		// 		)		
		// 		// m_shooterPivot.goToAngleCommand(45)
		// 		// m_simpleFlywheelBottom.pidCommand(2300),
		// 		// m_simpleFlywheelTop.pidCommand(2300)
		// 	)
		// ));
		



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
				() -> !m_controller.getHID().getAButton(), //|| !m_buttonPad.getHID().getRawButton(9),

				// Slow Mode
				// () -> m_controller.getHID().getXButton() || m_ClimberLeft.getMotorRotations() > 2 || m_ClimberRight.getMotorRotations() > 2,
				() -> m_controller.getHID().getXButton(),		
				

				// Disable X Movement
				() -> false,

				// Disable Y Movement
				() -> false,

				// Heading Override
				() -> {
					if (m_buttonPad.getHID().getRawButton(1)){
						if (m_camera1.doesCameraHaveTarget())
							return m_camera1.getRobotYaw();
						return null;
					}

					return null;
				},
				() -> {
					if (m_buttonPad.getHID().getRawButton(88)) {
						if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
							return 152.79;
						} else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
							return -152.79;
						}
					}
					
					if (m_controller.getHID().getLeftBumper()) {
						if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
							return 90.0;
						} else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
							return 90.0;
						}
					}

					if (m_controller.getHID().getRightStickButton()) {
						if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
							return -60.0;
						} else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
							return -120.0;
						}
					}
					if (m_controller.getHID().getRightBumper()) {
						if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
							return 162.0;
						} else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
							return 202.0+180.0
							;
						}
					}


					// switch (m_controller.getHID().getPOV())
					//   case 0: return 0.0;
					//   case 180: return 180.0;
					//   default: return null;
					// }
					return null;
				},
				true
			));

		m_controller.start().onTrue(new InstantCommand(
			m_drivetrain::resetHeading)); // TODO this should also do
		// something with odometry? As
		// it freaks out



		// m_controller.povRight().whileTrue(new RobotTurnToAngle(m_drivetrain, 0));

		// m_controller.povLeft().whileTrue(new RobotTurnToAngle(m_drivetrain, 180));
		new Trigger(() -> !m_brakeButton.get() && DriverStation.isDisabled()).whileTrue(new StartEndCommand(
			()-> {
				m_intakePivot.setBrakeMode(false);
				m_feeder.setBrakeMode(false);
				m_shooterPivot.setBrakeMode(false);
			},
			() -> {
				m_intakePivot.setBrakeMode(true);
				m_feeder.setBrakeMode(true);
				m_shooterPivot.setBrakeMode(true);
			}
		).ignoringDisable(true));

    
	}

	/**
	* Use this to pass the autonomous command to the main {@link Robot} class.
	*
	* @return the command to run in autonomous
	*/
	public Command getAutonomousCommand() {
		return m_autoSelector.selected();
	}
	public Command speakerShoot() {
		return new ParallelCommandGroup(
			m_shooterPivot.goToAngleCommand(59.92836363),
			m_simpleFlywheelBottom.pidCommand(4000),
			m_simpleFlywheelTop.pidCommand(4000),

			new SequentialCommandGroup(
				new WaitCommand(0.005), //wait for setpoint to change
				new WaitUntilCommand(
					() -> m_simpleFlywheelBottom.isErrorInRange() && m_simpleFlywheelTop.isErrorInRange() && m_shooterPivot.isAngleErrorInRange()),

				m_feeder.forward()
			)
		).until(()-> m_TOF2.isTOF1OutOfRange());
	}

	public Command speakerShoot2() {
		return new ParallelCommandGroup(
			m_shooterPivot.goToAngleCommand(59.92836363),
			m_simpleFlywheelBottom.pidCommand(3000),
			m_simpleFlywheelTop.pidCommand(3000),

			new SequentialCommandGroup(
				new WaitCommand(0.005), //wait for setpoint to change
				new WaitUntilCommand(
					() -> m_simpleFlywheelBottom.isErrorInRange() && m_simpleFlywheelTop.isErrorInRange() && m_shooterPivot.isAngleErrorInRange()),

				m_feeder.forward()
			)
		).until(()-> m_TOF2.isTOF1OutOfRange());
	}

	

	public Command ampShoot() {
		return new ParallelCommandGroup(
			m_shooterPivot.goToAngleCommand(52),//50
			m_simpleFlywheelBottom.pidCommand(1350),//1250
			m_simpleFlywheelTop.pidCommand(850),//750

			new SequentialCommandGroup(
				new WaitCommand(0.05),
				new WaitUntilCommand(() -> 
					m_simpleFlywheelBottom.isErrorInRange() && m_simpleFlywheelTop.isErrorInRange() && m_shooterPivot.isAngleErrorInRange()),
				m_feeder.forward()
			)
		);
	}

	
	public Command ampShoot2() {
		return new ParallelCommandGroup(
			m_shooterPivot.goToAngleCommand(52),//50
			m_simpleFlywheelBottom.pidCommand(1550),//1250
			m_simpleFlywheelTop.pidCommand(1050),//750

			new SequentialCommandGroup(
				new WaitCommand(0.05),
				new WaitUntilCommand(() -> 
					m_simpleFlywheelBottom.isErrorInRange() && m_simpleFlywheelTop.isErrorInRange() && m_shooterPivot.isAngleErrorInRange()),
				m_feeder.forward()
			)
		);
	}

	private double angle;
	private double rpm;

	public Command feed(){
		// return new SequentialCommandGroup(
		// new InstantCommand(()-> getAngleAndRPM()),
		//  new ParallelCommandGroup(
		// 	m_shooterPivot.goToAngleCommand(angle),
		// 	m_simpleFlywheelBottom.pidCommand(rpm),
		// 	m_simpleFlywheelTop.pidCommand(rpm),
		return m_feeder.forward();
		// ));
	}

	private void getAngleAndRPM() {
		angle = m_shooterPivot.getAngle();
		rpm = m_simpleFlywheelTop.getVelocity();
	}

	public Command StageShoot() {
		return new ParallelCommandGroup(
			m_shooterPivot.goToAngleCommand(34.56),
			m_simpleFlywheelBottom.pidCommand(3737),
			m_simpleFlywheelTop.pidCommand(3737),

			new SequentialCommandGroup(
				new WaitUntilCommand(() -> 
					m_simpleFlywheelBottom.isErrorInRange() && m_simpleFlywheelTop.isErrorInRange() && m_shooterPivot.isAngleErrorInRange()),
				m_feeder.forward()
			)
		);
	}

	public void autonomousInit() {
		m_drivetrain.resetHeading();
		m_drivetrain.setNominalVoltages(AutoConstants.kAutoVoltageCompensation);
		m_intakePivot.setBrakeMode(true);
		m_feeder.setBrakeMode(true);
		m_shooterPivot.setBrakeMode(true);
		m_drivetrain.enableBrakeMode();
	}
	public void teleopInit() {
		m_drivetrain.setNominalVoltages(DriveConstants.kDriveVoltageCompensation);
		m_intakePivot.setBrakeMode(true);
		m_feeder.setBrakeMode(true);
		m_shooterPivot.setBrakeMode(true);
		m_drivetrain.enableBrakeMode();
	}
	public void robotPeriodic() {
		SmartDashboard.putData(CommandScheduler.getInstance());
		SmartDashboard.putData(m_ClimberLeft);
		SmartDashboard.putData(m_ClimberRight);
		SmartDashboard.putData(m_simpleFlywheelBottom);
		SmartDashboard.putData(m_simpleFlywheelTop);
		SmartDashboard.putData(m_shooterPivot);
		SmartDashboard.putData(m_intake);
		SmartDashboard.putData(m_intakePivot);
		SmartDashboard.putData(m_feeder);
		SmartDashboard.putData(m_drivetrain);
		SmartDashboard.putNumber("Angle", ShooterPivotConstants.getAngle(m_camera1.FilteredEsimatedPoseNorm()));
		SmartDashboard.putNumber("RPM", FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm()));
		SmartDashboard.putBoolean("isYaw", isYawInRangeDebounced());
	}
	public void disabledPeriodic() {
		m_drivetrain.resetSteerEncoders();
	}

	private void addValuesToDashboard() {
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
	}
	private void setupShooter() {
		m_simpleFlywheelBottom.setDefaultCommand(m_simpleFlywheelBottom.pidCommand(4000));
		m_simpleFlywheelTop.setDefaultCommand(m_simpleFlywheelTop.pidCommand(4000));
		m_shooterPivot.setDefaultCommand(m_shooterPivot.goToAngleCommand(45));
		m_intakePivot.setDefaultCommand(m_intakePivot.stowCommand());
		m_shooterPivot.setDefaultCommand(
			new SequentialCommandGroup(
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitCommand(0.1),
						new WaitUntilCommand( ()-> m_shooterPivot.isAngleErrorInRange())
					),
					m_shooterPivot.goToAngleCommand(45)
				),
				new RunCommand(()-> {})
			)
		);

	}
	private void registerAutos(HashMap<String, String> autos) {
		for (String name: autos.keySet()) {
			m_autoSelector.registerCommand(
				name, name, m_drivetrain.createAutoPath(autos.get(name),
				AutoConstants.kPathConstraints
			));
		}
	}

	//
	// Commmand Groups
	//

	public boolean isYawInRange(){
		return (m_camera1.getRobotYaw() < 4 && m_camera1.getRobotYaw() > -4);
	}

	private final Debouncer m_debouncer = new Debouncer(.5, DebounceType.kBoth);

	public boolean isYawInRangeDebounced(){
		return m_debouncer.calculate(isYawInRange());
	}

	public Command visionShootAndXStop(){
		return new ParallelRaceGroup(
			m_shooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle((m_camera1.FilteredEsimatedPoseNorm()))),
			m_simpleFlywheelBottom.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
			m_simpleFlywheelTop.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
			// m_simpleFlywheelLeft.feederShooterCommand(m_SimpleShooterFeeder)
			new SequentialCommandGroup(
				new WaitCommand(0.25), //wait for setpoint to change
				new WaitUntilCommand(()-> isYawInRangeDebounced()),
				m_drivetrain.xStop().asProxy().until(
					() -> m_simpleFlywheelBottom.isErrorInRange() && m_simpleFlywheelTop.isErrorInRange() && m_shooterPivot.isAngleErrorInRange()
				),

				new ParallelRaceGroup(
					m_feeder.forward(), m_drivetrain.xStop().asProxy(),
					new SequentialCommandGroup(
						new WaitUntilCommand(()-> m_TOF1.isTOF1OutOfRange()),
						new WaitCommand(0.25)
					)
				)
			)

		);
		// ).until(() -> m_TOF1.isTOF1OutOfRange());
	}
	public Command visionShoot() {
		return new ParallelRaceGroup(
			new AlignYawWithTAG(m_drivetrain, m_camera1),
			m_shooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle((m_camera1.FilteredEsimatedPoseNorm()))),
			m_simpleFlywheelBottom.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
			m_simpleFlywheelTop.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
			// m_simpleFlywheelLeft.feederShooterCommand(m_SimpleShooterFeeder)
			new SequentialCommandGroup(
				new WaitCommand(0.005), //wait for setpoint to change
				new WaitUntilCommand(()-> isYawInRangeDebounced()), 
				new WaitUntilCommand(() -> 
					m_simpleFlywheelBottom.isErrorInRange() && m_simpleFlywheelTop.isErrorInRange() && m_shooterPivot.isAngleErrorInRange()),

					new ParallelRaceGroup(
						m_feeder.forward(),
						m_intake.test_forwardsCommand(),
						new SequentialCommandGroup(
							new WaitUntilCommand(()-> m_TOF2.isTOF1OutOfRange() && m_TOF1.isTOF1OutOfRange())
							// new WaitCommand(0.25)
						)
					)
					)
					);
	}
	public Command intakeCommand(){
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new WaitUntilCommand((()->m_TOF1.isTOF1WithinRange())),
				new WaitCommand(0)
			),
			new SequentialCommandGroup(
				m_intake.test_forwardsCommand().until(() -> m_intake.isRingIntaked()),
				// new WaitCommand(1.3),
				new WaitUntilCommand(()-> m_intakePivot.getAngle() > 50),
				new WaitCommand(0.2),
				m_intake.test_forwardsCommand()
			),
			m_shooterPivot.goToAngleCommand(45),
			new SequentialCommandGroup(
				m_intakePivot.deploy().until(() -> m_intake.isRingIntaked()),
				m_intakePivot.intakeLocation()
			),
			m_feeder.forward(),

			new SequentialCommandGroup(
				new WaitUntilCommand(() -> m_intake.isRingIntaked()),
				new StartEndCommand(
					() -> m_controller.getHID().setRumble(RumbleType.kBothRumble, .3),
					() -> m_controller.getHID().setRumble(RumbleType.kBothRumble, 0)
				).withTimeout(.2)
			)
		);
	}

	public Command intakingCommand(){
		return new ParallelDeadlineGroup(
			m_intake.test_forwardsCommand().until(() -> m_intake.isRingIntaked()),	
			new SequentialCommandGroup(
					new WaitUntilCommand(()-> m_intakePivot.getAngle() < 10),
					new AlignYawWithNote(m_drivetrain, m_controller,0.35).until(()-> m_intake.isRingIntaked()
			)),
			m_intakePivot.deploy()			
		);	
	}
	public Command intakeUpCommand(){
		return new ParallelDeadlineGroup(
			new WaitUntilCommand((()->m_TOF2.isTOF1WithinRange())),//deadline
			new SequentialCommandGroup(
				new WaitUntilCommand(()-> m_intakePivot.getAngle() > 87),
				m_intake.test_forwardsCommand()
			),
			// m_shooterPivot.goToAngleCommand(45),
			m_intakePivot.stowCommand(),
			new SequentialCommandGroup(
				m_feeder.forward().until(()-> m_TOF1.isTOF1WithinRange()),	
				m_feeder.forwardPercent(0.2).until(()-> m_TOF2.isTOF2WithinRange())
			)
		);
	}	
	public Command splitIntake(){
		return(new SequentialCommandGroup(
			intakingCommand(),
			intakeUpCommand()
		));
	}


	public Command intakeCommand2(){
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new WaitUntilCommand((() -> m_TOF2.isTOF1WithinRange()))
			),
			new SequentialCommandGroup(
				m_intake.test_forwardsCommand().until(() -> m_intake.isRingIntaked() && m_intakePivot.getAngle() < 10),
				// new WaitUntilCommand(()-> m_intakePivot.getAngle() > 100),
				new WaitUntilCommand(()-> m_intakePivot.getAngle() > 100),
				m_intake.intakePercentCommand(4)
			),
			new SequentialCommandGroup(
				m_intakePivot.deploy().until(() -> m_intake.isRingIntaked() && m_intakePivot.getAngle() < 10),
				m_intakePivot.stowCommand()
			),
			new SequentialCommandGroup(
				m_feeder.forward().until(()-> m_TOF1.isTOF1WithinRange()),	
				m_feeder.forward().withTimeout(0.05),
				m_feeder.forwardPercent(0.2).until(()-> m_TOF2.isTOF2WithinRange())
			),
			new SequentialCommandGroup(
				new WaitUntilCommand(() -> m_intake.isRingIntaked() && m_intakePivot.getAngle() < 10),
				new StartEndCommand(
					() -> m_controller.getHID().setRumble(RumbleType.kBothRumble, .3),
					() -> m_controller.getHID().setRumble(RumbleType.kBothRumble, 0)
				).withTimeout(.2)
			)
		);
	}
	public Command intakeCommandAuto(){
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new WaitUntilCommand((() -> m_TOF2.isTOF1WithinRange()))
			),
			new SequentialCommandGroup(
				m_intake.test_forwardsCommand().until(() -> m_intake.isRingIntaked()),
				new WaitUntilCommand(()-> m_intakePivot.getAngle() > 87),
				m_intake.test_forwardsCommand()

			),
			new SequentialCommandGroup(
				m_intakePivot.deploy().until(() -> m_intake.isRingIntaked()),
				m_intakePivot.stowCommand()
			),
			new SequentialCommandGroup(
				m_feeder.forward().until(()-> m_TOF1.isTOF1WithinRange()),	
				m_feeder.forwardPercent(0.2).until(()-> m_TOF2.isTOF2WithinRange())
			),
			new SequentialCommandGroup(
				new WaitUntilCommand(() -> m_intake.isRingIntaked()),
				new StartEndCommand(
					() -> m_controller.getHID().setRumble(RumbleType.kBothRumble, .3),
					() -> m_controller.getHID().setRumble(RumbleType.kBothRumble, 0)
				).withTimeout(.2)
			),
			m_shooterPivot.goToAngleCommand(ShooterPivotConstants.getAngle((m_camera1.FilteredEsimatedPoseNorm())))
		);
	}	


	public Command extendClimber(){
		return new ParallelCommandGroup(m_ClimberLeft.extendCommand(), m_ClimberRight.extendCommand());
	}

	public Command retractClimber(){
		return new ParallelCommandGroup(m_ClimberLeft.retractCommand(), m_ClimberRight.retractCommand(), new ScheduleCommand( m_intakePivot.deploy()));
	}
	private Command autoShoot() {
		return visionShoot();
	}

	public void slowmodething(){
		slowMode = !slowMode;
	}
	public Command idleAuto(){
		return new ParallelCommandGroup(
			m_shooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle((m_camera1.FilteredEsimatedPoseNorm()))),
			m_simpleFlywheelBottom.pidCommand(()-> FlyWheelConstants.getRPM((m_camera1.FilteredEsimatedPoseNorm()))),
			m_simpleFlywheelTop.pidCommand(()-> FlyWheelConstants.getRPM((m_camera1.FilteredEsimatedPoseNorm())))
		);
	}
	public Command idleAuto2(double angle){
		return new ParallelCommandGroup(
			m_shooterPivot.goToAngleCommand(27),
			m_simpleFlywheelBottom.pidCommand(4000),
			m_simpleFlywheelTop.pidCommand(4000)
		);
	}
	
	public Command autoPickUp(){
		return new ParallelDeadlineGroup(
				intakeCommand2(),
				new SequentialCommandGroup(
					new WaitUntilCommand(()-> m_intakePivot.getAngle() < 30),
					new AlignYawWithNote(m_drivetrain, m_controller,0.45).until(()-> m_intake.isRingIntaked())
				));	
	}

	public Command passNote(){
		return new ParallelRaceGroup(
			m_shooterPivot.goToAngleCommand(45),
			m_simpleFlywheelBottom.pidCommand(3000),
			m_simpleFlywheelTop.pidCommand(3000),
			new SequentialCommandGroup(
				new WaitCommand(0.005), //wait for setpoint to change
				new WaitUntilCommand(() -> m_simpleFlywheelBottom.isErrorInRange() && m_simpleFlywheelTop.isErrorInRange() && m_shooterPivot.isAngleErrorInRange()),
				new ParallelRaceGroup(
					m_feeder.forward(),
					new SequentialCommandGroup(
						new WaitUntilCommand(()-> m_TOF2.isTOF1OutOfRange()),
						new WaitCommand(0.25)
					)
				)
			)

		);
		// ).until(() -> m_TOF1.isTOF1OutOfRange());
	}	
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.kennedyrobotics.auto.AutoSelector;
import com.kennedyrobotics.hardware.misc.RevDigit;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
			put("auto-shoot", new ParallelRaceGroup(
				autoShoot(), m_intakePivot.stowCommand()
			));
			put("pickup", intakeCommand().withTimeout(4));
			put("deploy-intake", m_intakePivot.deploy());
			put("Intake-up", m_intakePivot.stowCommand().until(()-> (m_intakePivot.getAngle() > 80)));
			put("IntakeP1", m_intaking());
			put("IntakeP2", intakeUpCommand());
		}});

		registerAutos(new HashMap<String, String>() {{
			//: basic autos - 2 note score
			put("2SRC", "2SRC");
			put("2CEN", "2CEN");
			put("2AMP", "2AMP");

			//: only moves - nothing else
			put("MOVE", "MOVE");

			//: single note autos - shoot only
			put("1SRC", "1SRC");
			put("1CEN", "1CEN");
			put("1AMP", "1AMP");
			
			//: extra autos - lots of notes
			put("FAR1", "FAR1");	
			put("FAR2", "FAR2");
			put("4SRC", "4SRC");	

			put("3SRC", "3SRC");
			put("3CEN", "3CEN");
			put("3AMP", "3AMP");
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
		m_controller.rightBumper().toggleOnTrue(new ParallelCommandGroup(
			m_Orchestra6.playMusicCommand(),
			// m_Orchestra6v21.playMusiCommand(),
			// m_Orchestra6v22.playMusiCommand(),
			// m_Orchestra6v23.playMusiCommand(),
			// m_Orchestra6v24.playMusiCommand(),
			m_simpleFlywheelBottom.pidCommand(0),
			m_simpleFlywheelTop.pidCommand(0)
		));

		m_controller.y().whileTrue(this.extendClimber());
		m_controller.b().whileTrue(this.retractClimber());
		m_buttonPad.button(8).whileTrue(m_shooterPivot.openLoopCommand(2));
		m_buttonPad.button(12).whileTrue(m_shooterPivot.openLoopCommand(-2));
		m_buttonPad.button(6).whileTrue(m_intakePivot.deploy());
		m_buttonPad.button(7).whileTrue(m_intakePivot.stowCommand());

		m_controller.povUp().onTrue(new InstantCommand(()-> m_camera1.offset+=1));
		m_controller.povDown().onTrue(new InstantCommand(()-> m_camera1.offset -=1));
		m_controller.povLeft().onTrue(new InstantCommand(()-> m_camera1.offset = 0));
		

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
			m_shooterPivot.goToAngleCommand(48.779296875),
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

		m_buttonPad.button(5).whileTrue(new ParallelCommandGroup(
			m_shooterPivot.goToAngleCommand(57.91),
			m_simpleFlywheelBottom.pidCommand(2300),
			m_simpleFlywheelTop.pidCommand(2300)
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
					if (m_buttonPad.getHID().getRawButton(8)) {
						if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
							return 152.79;
						} else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
							return -152.79;
						}
					}

				// switch (m_controller.getHID().getPOV())
				//   case 0: return 0.0;
				//   case 180: return 180.0;
				//   default: return null;
				// }
					return null;
				}
			));

		m_controller.start().onTrue(new InstantCommand(
			m_drivetrain::resetHeading)); // TODO this should also do
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
		return m_autoSelector.selected();
	}
	public Command speakerShoot() {
		return new ParallelCommandGroup(
			m_shooterPivot.goToAngleCommand(59.92836363),
			m_simpleFlywheelBottom.pidCommand(2326.626089),
			m_simpleFlywheelTop.pidCommand(2326.626089),

			new SequentialCommandGroup(
				new WaitCommand(0.25), //wait for setpoint to change
				new WaitUntilCommand(
					() -> m_simpleFlywheelBottom.isErrorInRange() && m_simpleFlywheelTop.isErrorInRange() && m_shooterPivot.isAngleErrorInRange()),

				m_feeder.forward()
			)
		).withTimeout(2);
	}

	public Command ampShoot() {
		return new ParallelCommandGroup(
			m_shooterPivot.goToAngleCommand(50),
			m_simpleFlywheelBottom.pidCommand(1250),
			m_simpleFlywheelTop.pidCommand(750),

			new SequentialCommandGroup(
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
	}
	public void teleopInit() {
		m_drivetrain.setNominalVoltages(DriveConstants.kDriveVoltageCompensation);
	}
	public void robotPeriodic() {
		SmartDashboard.putNumber("Angle", ShooterPivotConstants.getAngle(m_camera1.FilteredEsimatedPoseNorm()));
		SmartDashboard.putNumber("RPM", FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm()));
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
		m_simpleFlywheelBottom.setDefaultCommand(m_simpleFlywheelBottom.pidCommand(2000));
		m_simpleFlywheelTop.setDefaultCommand(m_simpleFlywheelTop.pidCommand(2000));
		// m_shooterPivot.setDefaultCommand(m_shooterPivot.goToAngleCommand(45));
		m_intakePivot.setDefaultCommand(m_intakePivot.stowCommand());
		// m_shooterPivot.setDefaultCommand(
		// 	new SequentialCommandGroup(
		// 		new ParallelDeadlineGroup(
		// 			new SequentialCommandGroup(
		// 				new WaitCommand(0.1),
		// 				new WaitUntilCommand( ()-> m_shooterPivot.isAngleErrorInRange())
		// 			),
		// 			m_shooterPivot.goToAngleCommand(45)
		// 		),
		// 		new RunCommand(()-> {})
		// 	)
		// )
		;
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
		return (m_camera1.getRobotYaw() < 2.5 && m_camera1.getRobotYaw() > -2.5);
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
			m_shooterPivot.goToAngleCommand(()-> ShooterPivotConstants.getAngle((m_camera1.FilteredEsimatedPoseNorm()))),
			m_simpleFlywheelBottom.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
			m_simpleFlywheelTop.pidCommand(()-> FlyWheelConstants.getRPM(m_camera1.FilteredEsimatedPoseNorm())),
			// m_simpleFlywheelLeft.feederShooterCommand(m_SimpleShooterFeeder)
			new SequentialCommandGroup(
				new WaitCommand(0.25), //wait for setpoint to change
				new WaitUntilCommand(() -> 
					m_simpleFlywheelBottom.isErrorInRange() && m_simpleFlywheelTop.isErrorInRange() && m_shooterPivot.isAngleErrorInRange()),

					new ParallelRaceGroup(
						m_feeder.forward(),
						m_intake.test_forwardsCommand(),
						new SequentialCommandGroup(
							new WaitUntilCommand(()->m_TOF1.isTOF1OutOfRange()),
							new WaitCommand(0.25)
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
		return new ParallelRaceGroup(
			//intake roll till ring detected by right sight
			m_intake.test_forwardsCommand().until(() -> m_intake.isRingIntaked()),	
			m_intakePivot.deploy()			
		);	
	}
	public Command intakeUpCommand(){
		return new ParallelDeadlineGroup(
			new WaitUntilCommand((()->m_TOF1.isTOF1WithinRange())),//deadline
			new SequentialCommandGroup(
				new WaitUntilCommand(()-> m_intakePivot.getAngle() > 55),
				m_intake.test_forwardsCommand()
			),
			m_shooterPivot.goToAngleCommand(45),
			m_intakePivot.intakeLocation(),
			m_feeder.forward()
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
				m_intake.test_forwardsCommand().until(() -> m_intake.isRingIntaked()),
				new WaitUntilCommand(()-> m_intakePivot.getAngle() > 87),
				m_intake.test_forwardsCommand()
			),
			m_shooterPivot.goToAngleCommand(45),
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
			)
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
}

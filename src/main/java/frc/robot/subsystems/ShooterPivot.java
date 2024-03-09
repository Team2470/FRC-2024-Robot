// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPivotConstants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class ShooterPivot extends SubsystemBase {
// private final int kOpenLoop = 0;
// private final int kPID = 1;
// private final int kStateSpace = 2;
private enum ControlMode {
	kOpenLoop,
	kPID,
	kMusicMode
}

private final WPI_TalonFX m_motor;
//private final CANSparkFlex m_follower;

private final CANCoder m_encoder;

private final Orchestra m_Orchestra;

private ControlMode m_controlMode = ControlMode.kOpenLoop;
private double m_demand;

	String[] m_songs = new String[] {
	"song1.chrp",//Never Gonna Give You Up
	"song2.chrp",//Star Sprangled Banner
	"song3.chrp",//Viva la vida
	"song4.chrp",//Star Sprangled Banner 2
	"song5.chrp",//Runaway
	"song6.chrp",//Interstellar
	"song7.chrp",//Waitng for love
	"song8.chrp",//never gonna give you up
	"song9.chrp",
	"song10.chrp",
	"song11.chrp",
	};

private int currentSong = 0;
//
// PID
//

private final ProfiledPIDController m_pidController = new ProfiledPIDController(ShooterPivotConstants.kP, ShooterPivotConstants.kI, ShooterPivotConstants.kD,
new TrapezoidProfile.Constraints(Math.toRadians(90), Math.toRadians(90)));

private ArmFeedforward m_Feedforward =
	new ArmFeedforward(0, ShooterPivotConstants.kG, ShooterPivotConstants.kV, ShooterPivotConstants.kA);

public ShooterPivot() {
	m_motor = new WPI_TalonFX(Constants.ShooterPivotConstants.MotorID, Constants.ShooterPivotConstants.MotorCANBus);
	m_motor.configFactoryDefault();
	m_motor.setInverted(true);
	m_motor.setNeutralMode(NeutralMode.Brake);

	m_encoder = new CANCoder(Constants.ShooterPivotConstants.EncoderID, Constants.ShooterPivotConstants.EncoderCANBus);
	m_encoder.configFactoryDefault();

	m_motor.configForwardSoftLimitEnable(true);
	m_motor.configReverseSoftLimitEnable(true);
	m_motor.configReverseSoftLimitThreshold(ShooterPivotConstants.reverseSoftLimit);
	m_motor.configForwardSoftLimitThreshold(ShooterPivotConstants.forwardSoftLimit);
	m_motor.setNeutralMode(NeutralMode.Brake);
	// set voltage dosen't work with voltage compensation
	// m_motor.configVoltageCompSaturation(10);
	// m_motor.enableVoltageCompensation(true);

	m_Orchestra = new Orchestra();
	m_Orchestra.addInstrument(m_motor);



	m_encoder.configFactoryDefault();
	m_encoder.configSensorDirection(ShooterPivotConstants.encoderDirection);
	m_encoder.configMagnetOffset(ShooterPivotConstants.encoderOffset);
	m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
	m_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

	m_motor.configRemoteFeedbackFilter(m_encoder, 0);
	m_motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
	m_motor.setSensorPhase(true);


	//
	// CAN Status Frames
	//
	// See https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html?highlight=status%20frame#setting-status-frame-periods
	//

	// Default 10ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
	// Default 20ms
	// TODO Change this to 10ms for better control?
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
	// Default 100ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
	// Default 100ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
	// Default ?ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
	// Default ?ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255);
	// Default 100ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
	// Default ?ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
	// Default 100ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
	// Default ?ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
	// Default 100ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
	// Default 100ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
	// Default 100ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
	// Default 100ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 255);
	// Default 50ms
	m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);



	m_Orchestra.loadMusic("song4.chrp");

	SmartDashboard.putNumber("SP kP", ShooterPivotConstants.kP);
	SmartDashboard.putNumber("SP kI", ShooterPivotConstants.kI);
	SmartDashboard.putNumber("SP kD", ShooterPivotConstants.kD);
	SmartDashboard.putNumber("SP kF", ShooterPivotConstants.kF);
	SmartDashboard.putNumber("SP kG", ShooterPivotConstants.kG);
	SmartDashboard.putNumber("SP kV", ShooterPivotConstants.kV);
	SmartDashboard.putNumber("SP kA", ShooterPivotConstants.kA);
	SmartDashboard.putString("song#", "song1");
}

public double getAngle() {
	return m_encoder.getAbsolutePosition();
}


@Override
public void periodic() {
	// Determine output voltage
	double outputVoltage = 0;

	switch (m_controlMode) {
	case kOpenLoop:
		// Do openloop stuff here
		outputVoltage = m_demand;
		break;

	case kPID:
		m_Feedforward = new ArmFeedforward(
		0,
		SmartDashboard.getNumber("SP kG", 0),
		SmartDashboard.getNumber("SP kV", 0),
		SmartDashboard.getNumber("SP kA", 0));

		// Do PID stuff
		m_pidController.setP(SmartDashboard.getNumber("SP kP", ShooterPivotConstants.kP));
		m_pidController.setI(SmartDashboard.getNumber("SP kI", ShooterPivotConstants.kI));
		m_pidController.setD(SmartDashboard.getNumber("SP kD", ShooterPivotConstants.kD));
		double kF = SmartDashboard.getNumber("SP kF", ShooterPivotConstants.kF);

		//outputVoltage = kF * Math.cos(Math.toRadians(m_demand)) + m_pidController.calculate(getAngle(), m_demand);
		double PIDoutPutVoltage = m_pidController.calculate(Math.toRadians(getAngle()), Math.toRadians(m_demand));
		double feedforwardVoltage = m_Feedforward.calculate(m_pidController.getSetpoint().position, m_pidController.getSetpoint().velocity);

		outputVoltage = PIDoutPutVoltage + feedforwardVoltage;
		SmartDashboard.putNumber("SP Pid output voltage", PIDoutPutVoltage);
		SmartDashboard.putNumber("SP Feed Fowrad output voltage", feedforwardVoltage);
		SmartDashboard.putNumber("SP PID Profile Position", Math.toDegrees(m_pidController.getSetpoint().position));
		break;
	case kMusicMode:
		m_Orchestra.play();
		// SmartDashboard.putNumeber("current song", m_songs[currentSong]);
		break;
	default:
		// What happened!?
		break;
	}

	if (m_controlMode != ControlMode.kMusicMode){
	m_motor.setVoltage(outputVoltage);
	// Do something with the motor
	}


	SmartDashboard.putNumber("Shooter Pivot" + " encoderAbosoluteAngle", m_encoder.getAbsolutePosition());
	SmartDashboard.putNumber("Shooter Pivot" + " encoderAngle", m_encoder.getPosition());
	SmartDashboard.putNumber("Shooter Pivot" + " Motor Selected Sensor position", m_motor.getSelectedSensorPosition());
	SmartDashboard.putNumber("Shooter Pivot" + " Motor Error", getErrorAngle());
	SmartDashboard.putNumber("Shooter Pivot" + " get measurement", getAngle());
	SmartDashboard.putNumber("shooter Pivot" + " Motor Output Voltage", outputVoltage);
	SmartDashboard.putNumber("shooter Pivot" + " Motor Setpoint Position", m_demand);
	SmartDashboard.putBoolean("Shooter Pivot" + "In range", isAngleErrorInRange());
}

public double getErrorAngle(){
	if (m_controlMode == ControlMode.kPID){
	return Math.toDegrees(m_pidController.getPositionError());
	}
	return 0;
}

public boolean isAngleErrorInRange(){
	if (m_controlMode == ControlMode.kPID){
	return (0.35 > getErrorAngle() && getErrorAngle() > -0.35);
	}
	return false;
}

public void setOutputVoltage(double OutputVoltage) {
	m_controlMode = ControlMode.kOpenLoop;
	m_demand = OutputVoltage;
}

public void setPIDSetpoint(double angleDegrees) {
	if (m_controlMode != ControlMode.kPID) {
	m_pidController.reset(Math.toRadians(getAngle()));
	}

	m_controlMode = ControlMode.kPID;
	m_demand = angleDegrees;
}

public void stop() {
	setOutputVoltage(0);
}


/**
* Example command factory method.
*
* @return a command
*/
public Command openLoopCommand(DoubleSupplier OutputVoltageSupplier) {


	// Inline construction of command goes here.
	// Subsystem::RunOnce implicitly requires `this` subsystem.
	return Commands.runEnd(
		() -> this.setOutputVoltage(OutputVoltageSupplier.getAsDouble()), this::stop, this);

}

public Command openLoopCommand(double OutputVoltage) {
	return openLoopCommand(()-> OutputVoltage);
}

public Command upWardCommand(double OutputVoltage) {
	return openLoopCommand(()-> 2);
}

public Command downWarCommand(double OutputVoltage) {
	return openLoopCommand(()-> -2);
}



public Command goToAngleCommand(DoubleSupplier angleSupplier){
	return Commands.runEnd(
	() -> this.setPIDSetpoint(angleSupplier.getAsDouble()), this::stop, this);
}

public Command goToAngleCommand(double angleDegrees){
	return goToAngleCommand(()-> angleDegrees);
}
public void nextSong(){
	currentSong = currentSong + 1;
}

public void backSong(){
	currentSong = currentSong - 1;
}

public void loadMusic(){
	if (currentSong < 0){
	m_Orchestra.loadMusic(m_songs[currentSong]);
	} else if (currentSong > m_songs.length){
	currentSong = 0;
	m_Orchestra.loadMusic(m_songs[currentSong]);
	} else {
	currentSong = 0;
	m_Orchestra.loadMusic(m_songs[currentSong]);
	}
}

public void playMusic(){
	// m_Orchestra.loadMusic(song + ".chrp");
	m_controlMode = ControlMode.kMusicMode;
}

public Command playMusiCommand(){
	return Commands.runEnd(
	() -> this.playMusic(), this::stop, this);
}

}

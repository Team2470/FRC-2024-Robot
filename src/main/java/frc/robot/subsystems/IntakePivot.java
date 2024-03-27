// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IntakePivotConstants;
import java.util.function.DoubleSupplier;


import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;



public class IntakePivot extends SubsystemBase {
private enum ControlMode {
	kOpenLoop,
	kPID
}


private final CANSparkMax m_motor;

private final CANCoder m_encoder;



private ControlMode m_controlMode = ControlMode.kOpenLoop;
private double m_demand;
private double Angle;
private double uplimit = 50;



//
// PID
//

private final PIDController m_pidController = new PIDController(IntakePivotConstants.kP, IntakePivotConstants.kI, IntakePivotConstants.kD);

public IntakePivot() {
	// m_motor = new WPI_TalonFX(Constants.IntakePivotConstants.MotorID, Constants.IntakePivotConstants.MotorCANBus);
	m_motor = new CANSparkMax(Constants.IntakePivotConstants.MotorID, MotorType.kBrushless);
	m_motor.restoreFactoryDefaults();
	m_motor.setInverted(true);
	m_motor.setOpenLoopRampRate(0.3);
	m_motor.setSmartCurrentLimit(40);
	m_motor.setIdleMode(IdleMode.kBrake);
	m_motor.setOpenLoopRampRate(0.25);


			// m_encoder = m_SimpleShooterFeeder.getEncoder();

	// Reduce CAN Bus usage, since we are using this as a dumb motor for week zero we can turn down
	// a lot of the status frame periods. When we start using the encoder, then we can increase the
	// kStatus2 frame back to 20ms (or 10ms)
	//
	// See ths page for what each frame contains: https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#can-packet-structure
	//
	// Default 10ms: Applied output, Faults, Sticky Faults, Is Follower
	m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
	// Default 20ms: Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
	m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
	// Default 20ms: Motor Position
	m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
	// Default 50ms: Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
	m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
	// Default 20ms: Alternate Encoder Velocity, Alternate Encoder Position
	m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
	// Default 200ms: Duty Cycle Absolute Encoder Position, Duty Cycle Absolute Encoder Absolute Angle
	m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
	// Default 200ms: Duty Cycle Absolute Encoder Velocity,  Duty Cycle Absolute Encoder Frequency
	m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
	// IDK what status 7 is, but I'm not going to touch it.
	// m_SimpleShooterFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 500);

	m_motor.burnFlash();


	// m_encoder = new CANCoder(Constants.IntakePivotConstants.EncoderID, Constants.IntakePivotConstants.EncoderCANBus);
	m_encoder = new CANCoder(Constants.IntakePivotConstants.EncoderID, Constants.IntakePivotConstants.EncoderCANBus);
	m_encoder.configFactoryDefault();





	m_encoder.configFactoryDefault();
	m_encoder.configSensorDirection(IntakePivotConstants.encoderDirection);
	m_encoder.configMagnetOffset(IntakePivotConstants.encoderOffset);
	m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
	m_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

	//Tells to use the non-motor encoder
}



public double getAngle() {
	return m_encoder.getAbsolutePosition();
}


@Override
public void periodic() {


	// Determine output voltage
	double outputVoltage = 0;
	double Angle = getAngle();
	switch (m_controlMode) {
	case kOpenLoop:
		// Do openloop stuff here
		outputVoltage = m_demand;
		break;

	case kPID:
		// Do PID stuff
		outputVoltage = m_pidController.calculate(getAngle(), m_demand);

		break;
	default:
		// What happened!?
		break;
	}

	if (Angle <= 10 && outputVoltage < 0 || Angle >= uplimit && outputVoltage > 0 ){
	outputVoltage = 0;
	}



	// Do something with the motor
	m_motor.setVoltage(outputVoltage);

	SmartDashboard.putNumber("Intake Pivot encoderAbosoluteAngle", m_encoder.getAbsolutePosition());
	SmartDashboard.putNumber("Intake Pivot encoderAngle", m_encoder.getPosition());
	// SmartDashboard.putNumber("Intake Pivot Motor Selected Sensor position", m_motor.getSelectedSensorPosition());
	SmartDashboard.putNumber("Intake Pivot Motor Error", getErrorAngle());
	SmartDashboard.putNumber("Intake Pivot Get Measurement", getAngle());
	SmartDashboard.putNumber("Intake Pivot Motor Output Voltage", outputVoltage);
	SmartDashboard.putNumber("Intake Pivot Motor Setpoint Position", m_demand);
	SmartDashboard.putNumber("Intake Pivot current", m_motor.getOutputCurrent());
}

public boolean isAtRetractedLimit(){
	return (getAngle() >= 90);
}

public boolean isAtExtentedLimit(){
	return(getAngle() <= 1);
}

public double getErrorAngle(){
	if (m_controlMode == ControlMode.kPID){
	return m_pidController.getPositionError();
	}
	return 0;
}

public void setOutputVoltage(double OutputVoltage) {
	m_controlMode = ControlMode.kOpenLoop;
	m_demand = OutputVoltage;
}

public void setPIDSetpoint(double angleDegrees) {
	m_controlMode = ControlMode.kPID;
	m_demand = angleDegrees;
}

public void stop() {
	setOutputVoltage(0);
}

public void setBrakeMode (boolean enabled) {
	if(enabled) {
		m_motor.setIdleMode(IdleMode.kBrake);
		
	} else {
		m_motor.setIdleMode(IdleMode.kCoast);
	}
}

/**
* Example command factory method.
*
* @return a command
*/
private Command openLoopCommand(DoubleSupplier OutputVoltageSupplier) {


	// Inline construction of command goes here.
	// Subsystem::RunOnce implicitly requires `this` subsystem.
	return Commands.runEnd(
		() -> this.setOutputVoltage(OutputVoltageSupplier.getAsDouble()), this::stop, this);

}

private Command openLoopCommand(double OutputVoltage) {
	return openLoopCommand(()-> OutputVoltage);
}

public Command stowCommand() {
	return new SequentialCommandGroup(
	new InstantCommand(()-> uplimit = 90),
	openLoopCommand(()-> 4).until(()->getAngle() > 60),
	openLoopCommand(()-> 1.5).until(()-> getAngle() > 110),
	openLoopCommand(0.3).until(()-> getAngle() > 115)
	);
}

public Command intakeLocation() {
	return new SequentialCommandGroup(
	new InstantCommand(()-> uplimit = 55),
	openLoopCommand(()-> 6)
	);
}

public Command deploy() {
	return new SequentialCommandGroup(
		openLoopCommand(()-> -3).until(()-> getAngle() < 10),
		openLoopCommand(-0.25)
	);
}



public Command goToAngle(DoubleSupplier angleSupplier){
	return Commands.runEnd(
	() -> this.setPIDSetpoint(angleSupplier.getAsDouble()), this::stop, this);
}

public Command goToAngle(double angleDegrees){
	return goToAngle(()-> angleDegrees);
}
}

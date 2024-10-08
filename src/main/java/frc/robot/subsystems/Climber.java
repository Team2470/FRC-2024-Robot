// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Climber extends SubsystemBase {

private final TalonFX m_motor ;
private final VoltageOut m_motorRequest = new VoltageOut(0);

private final Servo m_Servo;
private final DigitalInput m_ExtendLimit;
private final DigitalInput m_RetractLimit;
private final boolean m_isLeft;
private boolean m_homed;


/** Creates a new ClimberPivot. */
public Climber(int motorID, int servoChannel, int extendLimitChannel, int retractLimitChannel, boolean isLeft) {
	m_isLeft = isLeft;
	TalonFXConfiguration config = new TalonFXConfiguration();
	if (isLeft){
	config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
	} else {
	config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
	}
	config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
	config.CurrentLimits.SupplyCurrentLimit = 20;
	config.CurrentLimits.SupplyCurrentLimitEnable = true;
	config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
	config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 34;
	config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
	config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
	config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;


	m_motor = new TalonFX(motorID, "rio");
	m_motor.getConfigurator().apply(config);
	//m_motor.setInverted(isLeft);

	m_motor.getPosition().setUpdateFrequency(50);
	m_motor.optimizeBusUtilization();


	m_Servo = new Servo(servoChannel);
	m_ExtendLimit = new DigitalInput(extendLimitChannel);
	m_RetractLimit = new DigitalInput(retractLimitChannel);


}

public boolean isAtExtendLimit(){
	return !m_ExtendLimit.get();
}

public boolean isAtRetractLimit() {
	return !m_RetractLimit.get();
}

public double getMotorRotations() {
	return m_motor.getPosition().getValue();
}

@Override
public void periodic() {
	// This method will be called once per scheduler run
	SmartDashboard.putBoolean("Climber "+(m_isLeft? "Left":"Right")+" Is Extended", isAtExtendLimit());
	SmartDashboard.putBoolean("Climber "+(m_isLeft? "Left":"Right")+" Is Retracted", isAtRetractLimit());
	SmartDashboard.putBoolean("Climber "+(m_isLeft? "Left":"Right")+" Is Homed", m_homed);
	SmartDashboard.putNumber("Climber "+(m_isLeft?"Left":"Right")+ " Position", m_motor.getPosition().getValue());

	if (!m_homed && isAtRetractLimit())  {
	m_motor.setPosition(0);
	m_homed = true;
	}

}

public void engageRatchet(){
	if(m_isLeft){
	m_Servo.setPosition(0.35);
	}
	else{
	m_Servo.setPosition(0.475);
	}

}
public void disengageRatchet(){
	if(m_isLeft){
	m_Servo.setPosition(0.475);
	}
	else{
	m_Servo.setPosition(0.35);
	}
}

public void setVoltage(double voltage){
	if(!m_homed && voltage > 0){
	voltage = 0;
	}

	m_motorRequest.withOutput(voltage)
	.withLimitForwardMotion(isAtExtendLimit())
	.withLimitReverseMotion(isAtRetractLimit());
	m_motor.setControl(m_motorRequest);
}

public void stop(){
	setVoltage(0);
}

public Command retractCommand(){
	return new ParallelCommandGroup(
	Commands.run(() -> engageRatchet()),
	Commands.runEnd(() -> this.setVoltage(-3), this::stop, this));
}

	public Command extendCommand(){
	return new ParallelCommandGroup(
	Commands.run(() -> disengageRatchet()),
	new SequentialCommandGroup(
		new WaitCommand(0.2),
		Commands.runEnd(() -> this.setVoltage(4), this::stop, this)
	)
	);
}

}

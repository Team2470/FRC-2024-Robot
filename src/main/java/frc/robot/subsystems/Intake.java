package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Intake extends SubsystemBase {
	private final PWMSparkMax m_intake;

	private DigitalInput m_rightSight;

	public Intake() {
		m_intake = new PWMSparkMax(IntakeConstants.MotorID);

		m_intake.setInverted(false);

		m_rightSight = new DigitalInput(0);


	}

public boolean isRingIntaked(){
	return !m_rightSight.get();
	}

@Override
public void periodic() {
	SmartDashboard.putBoolean("Intake Right Sight", isRingIntaked());
}
public void intake() {
	m_intake.setVoltage(12);
}
public void reverse_intake () {
	m_intake.setVoltage(-12);

}
public void stop() {
	m_intake.stopMotor();
}
public Command test_forwardsCommand() {
	return Commands.runEnd(
		()-> this.intake(),
		this::stop,
		this);
}
	public Command test_reverseCommand() {
		return Commands.runEnd(
		()-> this.reverse_intake(),
		this::stop,
		this);
}
}

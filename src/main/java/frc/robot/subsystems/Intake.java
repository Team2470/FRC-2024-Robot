package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Intake extends SubsystemBase {
	private final PWMSparkMax m_intake;
	private final Debouncer m_debouncer = new Debouncer(.1, DebounceType.kBoth);


	// private DigitalInput m_rightSight;
	private final AnalogInput m_beamBreak;
	private final AnalogTrigger m_breamBreakTrigger;

	public Intake() {
		m_intake = new PWMSparkMax(IntakeConstants.MotorID);

		m_intake.setInverted(false);

		m_beamBreak = new AnalogInput(0);
		m_beamBreak.setAverageBits(2);
		m_breamBreakTrigger = new AnalogTrigger(m_beamBreak);
		m_breamBreakTrigger.setLimitsVoltage(1.5, 4);
	}

public boolean isRingIntaked(){
	return m_debouncer.calculate(!m_breamBreakTrigger.getTriggerState());
}

@Override
public void periodic() {
	SmartDashboard.putBoolean("Intake Right Sight", isRingIntaked());
}
public void intake() {
	m_intake.setVoltage(8);
}
public void reverse_intake () {
	m_intake.setVoltage(-8);

}
public void intakePercet(double volt){
	m_intake.setVoltage(volt);
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
public Command intakePercentCommand(double volt){
		return Commands.runEnd(
		()-> this.intakePercet(volt),
		this::stop,
		this);	
}
}

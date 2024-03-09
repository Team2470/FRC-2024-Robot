package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;


public class SimpleShooterFeeder extends SubsystemBase {
	private final CANSparkMax m_SimpleShooterFeeder;
	// public final RelativeEncoder m_encoder;

	public SimpleShooterFeeder(int canID) {
		m_SimpleShooterFeeder = new CANSparkMax(canID, MotorType.kBrushed);
		m_SimpleShooterFeeder.restoreFactoryDefaults();
		m_SimpleShooterFeeder.setInverted(false);
		m_SimpleShooterFeeder.setSmartCurrentLimit(40);
		// m_encoder = m_SimpleShooterFeeder.getEncoder();

		// Reduce CAN Bus usage, since we are using this as a dumb motor for week zero we can turn down
		// a lot of the status frame periods. When we start using the encoder, then we can increase the
		// kStatus2 frame back to 20ms (or 10ms)
		//
		// See ths page for what each frame contains: https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#can-packet-structure
		//
		// Default 10ms: Applied output, Faults, Sticky Faults, Is Follower
		m_SimpleShooterFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
		// Default 20ms: Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
		m_SimpleShooterFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
		// Default 20ms: Motor Position
		m_SimpleShooterFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
		// Default 50ms: Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
		m_SimpleShooterFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
		// Default 20ms: Alternate Encoder Velocity, Alternate Encoder Position
		m_SimpleShooterFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
		// Default 200ms: Duty Cycle Absolute Encoder Position, Duty Cycle Absolute Encoder Absolute Angle
		m_SimpleShooterFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
		// Default 200ms: Duty Cycle Absolute Encoder Velocity,  Duty Cycle Absolute Encoder Frequency
		m_SimpleShooterFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
		// IDK what status 7 is, but I'm not going to touch it.
		// m_SimpleShooterFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 500);

		m_SimpleShooterFeeder.burnFlash();
	}

	public double getVelocity() {
		// return m_encoder.getVelocity();
		return 0;
	}
	public double getEncoderPosition() {
		// return m_encoder.getPosition();
		return 0;
	}

	public double getEncoderCPR() {
		// return m_encoder.getCountsPerRevolution();
		return 0;
	}

	public boolean isEncoderPast5Rotations() {
		return (getEncoderPosition() > 5 || getEncoderPosition() < -5);
	}
	public void zeroEncoderValue() {
		// m_encoder.setPosition(0);
	}




	@Override
	public void periodic() {
		SmartDashboard.putNumber("Feeder Velocity", getVelocity());
		SmartDashboard.putNumber("Encoder Position", getEncoderPosition());
		SmartDashboard.putNumber("Encoder Counts Per Revolution", getEncoderCPR());
		SmartDashboard.putBoolean("Is Encoder Past 5 Rotations", isEncoderPast5Rotations());
	}


	public void feedShooter() {
		m_SimpleShooterFeeder.setVoltage(12);
	}
	public void reverseFeeder() {
		m_SimpleShooterFeeder.setVoltage((-12));
	}

	public void stopFeeder() {
		m_SimpleShooterFeeder.stopMotor();

	}

	public void feederForwards(double volts) {
		m_SimpleShooterFeeder.setVoltage(volts);
	}
	public void feederReverse(double volts) {
		m_SimpleShooterFeeder.setVoltage(volts);
	}


	public Command forward() {
		return Commands.runEnd(
			() -> this.feedShooter(),
			this::stopFeeder,
			this);
	}


	public Command reverse() {
		return Commands.runEnd(
			()-> this.reverseFeeder(),
			this::stopFeeder,
			this);
	}

	public Command stop() {
		return Commands.runEnd(() -> this.stopFeeder(), this::stopFeeder, this);
	}

	public Command resetEncoder() {
		return Commands.runOnce(() -> this.zeroEncoderValue(), this);
	}

	public Command forward(DoubleSupplier volts) {
		return Commands.runEnd(
			()-> this.feederForwards(volts.getAsDouble()),
			this::stopFeeder,
			this);
	}

	public Command reverse(DoubleSupplier volts) {
		return Commands.runEnd(
			()-> this.feederReverse(volts.getAsDouble()),
			this::stopFeeder,
			this);
	}
}

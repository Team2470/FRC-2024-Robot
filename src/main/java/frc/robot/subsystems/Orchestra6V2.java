// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Orchestra6V2 extends SubsystemBase {
/** Creates a new Orchestra6. */
	private final TalonFX m_motor;

	double stableVal;

	private final Orchestra m_Orchestra;

	public Orchestra6V2(int MotorID) {
		m_motor = new TalonFX(MotorID, "rio");
		m_Orchestra = new Orchestra();
		m_Orchestra.addInstrument(m_motor);

		// var status = m_Orchestra.loadMusic("song4.chrp");
		m_Orchestra.loadMusic("song4.chrp");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void playMusic() {
		// m_Orchestra.loadMusic(song + ".chrp");
		m_Orchestra.play();
	}

	public Command playMusiCommand() {
		return Commands.runEnd(() -> this.playMusic(), this::stop, this);
	}

	public void stop() {
		m_Orchestra.stop();
	}
}

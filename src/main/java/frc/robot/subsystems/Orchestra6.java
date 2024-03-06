// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Orchestra6 extends SubsystemBase {
  /** Creates a new Orchestra6. */
  private final TalonFX m_motor1 ;
  private final TalonFX m_motor2 ;
  private final TalonFX m_motor3 ;
  private final TalonFX m_motor4 ;

  private final Orchestra m_Orchestra;

  public Orchestra6(int MotorID, int MotorID2, int MotorID3, int MotorID4) {

    m_motor1 = new TalonFX(MotorID, "rio");
    m_motor2 = new TalonFX(MotorID2, "rio");
    m_motor3 = new TalonFX(MotorID3, "rio");
    m_motor4 = new TalonFX(MotorID4, "rio");

    m_Orchestra = new Orchestra();

    m_Orchestra.addInstrument(m_motor1);
    m_Orchestra.addInstrument(m_motor2);
    m_Orchestra.addInstrument(m_motor3);
    m_Orchestra.addInstrument(m_motor4);
    m_Orchestra.loadMusic("song4.chrp");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void playMusic(){
    // m_Orchestra.loadMusic(song + ".chrp");
    m_Orchestra.play();
  }

  public Command playMusiCommand(){
    return Commands.runEnd(
      () -> this.playMusic(), this::stop, this);
  }
  public void stop() {
    m_Orchestra.stop();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final WPI_TalonFX m_motor ;
  
  private final Servo m_Servo;


  /** Creates a new ClimberPivot. */
  public Climber(int motorID, int servoChannel, boolean isLeft) {

    m_motor = new WPI_TalonFX(motorID, "rio");
    m_motor.setInverted(isLeft);
    m_motor.configFactoryDefault();
    

    
    m_Servo = new Servo(servoChannel);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void up(){
    
  }
  
  public void down(){

  }

  public void stop(){

  }

  public Command upCommand(){
    return Commands.runEnd(
      () -> this.up(), this::stop, this);
  }

  public Command downCommand(){
    return Commands.runEnd(
      () -> this.down(), this::stop, this);
  }

}

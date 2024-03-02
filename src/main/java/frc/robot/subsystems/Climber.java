// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final TalonFX m_motor ;
  private final VoltageOut m_motorRequest = new VoltageOut(0);
  
  private final Servo m_Servo;


  /** Creates a new ClimberPivot. */
  public Climber(int motorID, int servoChannel, boolean isLeft) {

    TalonFXConfiguration config = new TalonFXConfiguration();
    if (isLeft){
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;      
    } else {
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;        
    }
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    
    m_motor = new TalonFX(motorID, "rio");
    m_motor.getConfigurator().apply(config);
    //m_motor.setInverted(isLeft);
  
    //m_motor.getPosition().setUpdateFrequency(50);
    m_motor.optimizeBusUtilization();

    
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

  public void setVoltage(double voltage){
    m_motorRequest.withOutput(voltage);
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

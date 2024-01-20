// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FlyWheelConstants;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class SimpleFlywheel extends SubsystemBase {
  // private final int kOpenLoop = 0;
  // private final int kPID = 1;
  // private final int kStateSpace = 2;
  private enum ControlMode {
    kOpenLoop, 
    kPID,
    kStateSpace
  }

  private final CANSparkFlex m_leader;
  //private final CANSparkFlex m_follower;

  private final RelativeEncoder m_encoder;
  private final boolean m_isLeft;

  private ControlMode m_controlMode = ControlMode.kOpenLoop;
  private double m_demand;

  //
  // PID
  //

  private final PIDController m_pidController = new PIDController(FlyWheelConstants.kP, FlyWheelConstants.kI, FlyWheelConstants.kD);

  public SimpleFlywheel(int canID, boolean isLeft) {
    m_leader = new CANSparkFlex(canID, MotorType.kBrushless);
    m_leader.restoreFactoryDefaults();
  

    m_leader.setInverted(!isLeft);
    m_isLeft = isLeft;

    m_leader.setSmartCurrentLimit(40);


    m_encoder = m_leader.getEncoder();
    m_leader.burnFlash();
    //m_follower.burnFlash();
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
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
        // Do PID stuff 
        outputVoltage = FlyWheelConstants.kF * m_demand + m_pidController.calculate(getVelocity(), m_demand);
        
        break;
      case kStateSpace:
        // Do statespace stuff here
        break;
      default:
        // What happened!?
        break;
    }


    // Do something with the motor    
    m_leader.setVoltage(outputVoltage);

    
    // Publish to smart dashboard
    SmartDashboard.putNumber("Flywheel " + (m_isLeft ? "Left" : "Right")+" Velocity", getVelocity());
    SmartDashboard.putNumber("Flywheel "+ (m_isLeft ? "Left" : "Right") +  " output voltage", outputVoltage);
  }

  public void setOutputVoltage(double OutputVoltage) {
    m_controlMode = ControlMode.kOpenLoop;
    m_demand = OutputVoltage;
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
}

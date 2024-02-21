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
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;


public class SimpleFlywheel extends SubsystemBase {
  // private final int kOpenLoop = 0;
  // private final int kPID = 1;
  // private final int kStateSpace = 2;
  private enum ControlMode { 
    kOpenLoop, kPID, kStateSpace
  }

  private final CANSparkFlex m_leader;
  // private final CANSparkFlex m_follower;

  private final RelativeEncoder m_encoder;
  private final boolean m_isLeft;

  private ControlMode m_controlMode = ControlMode.kOpenLoop;
  private double m_demand;

  //: PID
  private final PIDController m_pidController = new PIDController(FlyWheelConstants.kP, FlyWheelConstants.kI, FlyWheelConstants.kD);

  public SimpleFlywheel(int canID, boolean isLeft) {
    m_leader = new CANSparkFlex(canID, MotorType.kBrushless);
    m_leader.restoreFactoryDefaults();
  

    m_leader.setInverted(!isLeft);
    m_isLeft = isLeft;

    m_leader.setOpenLoopRampRate(0.2);
    m_leader.setSmartCurrentLimit(40);


    m_encoder = m_leader.getEncoder();

    // Reduce CAN Bus usage, we only care about kStatus0? and kStatus1.
    // TODO Think about chaning kStatus1 to 10ms, may make PID better? As
    // It would reduce measurement delay.
    //
    // See ths page for what each frame contains: https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#can-packet-structure
    //
    // Default 10ms: Applied output, Faults, Sticky Faults, Is Follower
    m_leader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); 
    // Default 20ms: Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
    m_leader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    // Default 20ms: Motor Position
    m_leader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    // Default 50ms: Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
    m_leader.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    // Default 20ms: Alternate Encoder Velocity, Alternate Encoder Position
    m_leader.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    // Default 200ms: Duty Cycle Absolute Encoder Position, Duty Cycle Absolute Encoder Absolute Angle
    m_leader.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    // Default 200ms: Duty Cycle Absolute Encoder Velocity,  Duty Cycle Absolute Encoder Frequency
    m_leader.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
    // IDK what status 7 is, but I'm not going to touch it.
    // m_SimpleShooterFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 500);

    m_leader.burnFlash();
    //m_follower.burnFlash();
  }

  public double getVelocity() {
    return m_encoder.getVelocity()*(30.0/18.0);
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
        m_pidController.setP(SmartDashboard.getNumber("kP", FlyWheelConstants.kP));
        m_pidController.setI(SmartDashboard.getNumber("kI", FlyWheelConstants.kI));
        m_pidController.setD(SmartDashboard.getNumber("kD", FlyWheelConstants.kD));
        double kF = SmartDashboard.getNumber("kF", FlyWheelConstants.kF);

        // Do PID stuff 
        outputVoltage = kF * m_demand + m_pidController.calculate(getVelocity(), m_demand);
        
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
    SmartDashboard.putNumber("Flywheel "+ (m_isLeft ? "Left" : "Right") + " RPM Error", getErrorRPM());
    SmartDashboard.putNumber("Flywheel " + (m_isLeft ? "Left" : "Right") + " RPM Percent Error", getErrorPercent());
    SmartDashboard.putString("Flywheel "+ (m_isLeft ? "Left" : "Right")+ " Control Mode", m_controlMode.toString());
    SmartDashboard.putBoolean("Flywheel InRange"+ (m_isLeft ? "Left" : "Right"), isErrorInRange());

  }

  public double getErrorRPM(){
    if (m_controlMode == ControlMode.kPID){
      return m_pidController.getPositionError();
    }
    return 0;
  }

  public double getErrorPercent(){
    if (m_controlMode == ControlMode.kPID){
      return (m_demand - m_encoder.getVelocity()) / m_demand * 10;
    }

    return 0;
  }
  public boolean isErrorInRange() {
    return (-5 < this.getErrorPercent() && this.getErrorPercent() < 5);
}
public Command waitUntilErrorInrange(){
  return Commands.waitUntil(()-> this.isErrorInRange());
}

  public boolean isErrorOutOfRange() {
    return (this.getErrorPercent() > 15);
}

public Command waitUntilErrorOutOfRange(){
  return Commands.waitUntil(() -> this.isErrorOutOfRange());
  
}

  public void setOutputVoltage(double OutputVoltage) {
    m_controlMode = ControlMode.kOpenLoop;
    m_demand = OutputVoltage;
  }
 
  public void setPIDSetpoint(double rpm) {
    m_controlMode = ControlMode.kPID;
    m_demand = rpm;
  }

  public void stop() {
    setOutputVoltage(0);
  }
  public Command feederShooterCommand(SimpleShooterFeeder m_feeder) {
    return Commands.repeatingSequence(
        this.waitUntilErrorInrange(),
        m_feeder.forward().until(()->this.isErrorOutOfRange())
    );
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


  public Command pidCommand(DoubleSupplier rpmSupplier){
    return Commands.runEnd(
      () -> this.setPIDSetpoint(rpmSupplier.getAsDouble()), this::stop, this);
  }

  public Command pidCommand(double rpm){
    return pidCommand(() -> rpm);
  }
}

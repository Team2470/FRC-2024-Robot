// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.Constants.ShooterPivotConstants;

import java.util.ResourceBundle.Control;
import java.util.function.DoubleSupplier;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;



public class ShooterPivot extends SubsystemBase {
  // private final int kOpenLoop = 0;
  // private final int kPID = 1;
  // private final int kStateSpace = 2;
  private enum ControlMode {
    kOpenLoop, 
    kPID
  }

  private final WPI_TalonFX m_motor;
  //private final CANSparkFlex m_follower;

  private final CANCoder m_encoder;

  private ControlMode m_controlMode = ControlMode.kOpenLoop;
  private double m_demand;

  //
  // PID
  //

  private final PIDController m_pidController = new PIDController(ShooterPivotConstants.kP, ShooterPivotConstants.kI, ShooterPivotConstants.kD);

  public ShooterPivot() {
    m_motor = new WPI_TalonFX(Constants.ShooterPivotConstants.MotorID, Constants.ShooterPivotConstants.MotorCANBus);
    m_motor.configFactoryDefault();
    m_motor.setInverted(true);
    m_motor.setNeutralMode(NeutralMode.Brake);

    m_encoder = new CANCoder(Constants.ShooterPivotConstants.EncoderID, Constants.ShooterPivotConstants.EncoderCANBus);
    m_encoder.configFactoryDefault();

    m_motor.configForwardSoftLimitEnable(true);
    m_motor.configReverseSoftLimitEnable(true);
    m_motor.configReverseSoftLimitThreshold(ShooterPivotConstants.reverseSoftLimit);
    m_motor.configForwardSoftLimitThreshold(ShooterPivotConstants.forwardSoftLimit);
    m_motor.setNeutralMode(NeutralMode.Brake);
    // set voltage dosen't work with voltage compensation
    // m_motor.configVoltageCompSaturation(10);
    // m_motor.enableVoltageCompensation(true);
    


    m_encoder.configFactoryDefault();
    m_encoder.configSensorDirection(ShooterPivotConstants.encoderDirection);
    m_encoder.configMagnetOffset(ShooterPivotConstants.encoderOffset);
    m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    m_motor.configRemoteFeedbackFilter(m_encoder, 0);
    m_motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    m_motor.setSensorPhase(true);

    SmartDashboard.putNumber("SP kP", FlyWheelConstants.kP);
    SmartDashboard.putNumber("SP kI", FlyWheelConstants.kI);
    SmartDashboard.putNumber("SP kD", FlyWheelConstants.kD);
    SmartDashboard.putNumber("SP kF", FlyWheelConstants.kF);
  }

  public double getAngle() {
    return m_encoder.getAbsolutePosition();
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
        m_pidController.setP(SmartDashboard.getNumber("SP kP", ShooterPivotConstants.kP));
        m_pidController.setI(SmartDashboard.getNumber("SP kI", ShooterPivotConstants.kI));
        m_pidController.setD(SmartDashboard.getNumber("SP kD", ShooterPivotConstants.kD));
        double kF = SmartDashboard.getNumber("SP kF", ShooterPivotConstants.kF);

        outputVoltage = kF * Math.cos(Math.toRadians(m_demand)) + m_pidController.calculate(getAngle(), m_demand);
        
        break;
      default:
        // What happened!?
        break;
    }


    // Do something with the motor    
    m_motor.setVoltage(outputVoltage);

    SmartDashboard.putNumber("Shooter Pivot" + " encoderAbosoluteAngle", m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Shooter Pivot" + " encoderAngle", m_encoder.getPosition());
    SmartDashboard.putNumber("Shooter Pivot" + " Motor Selected Sensor position", m_motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shooter Pivot" + " Motor Error", getErrorAngle());
    SmartDashboard.putNumber("Shooter Pivot" + " get measurement", getAngle());
    SmartDashboard.putNumber("shooter Pivot" + " Motor Output Voltage", outputVoltage);
    SmartDashboard.putNumber("shooter Pivot" + " Motor Setpoint Position", m_demand);
  }

  public double getErrorAngle(){
    if (m_controlMode == ControlMode.kPID){
      return m_pidController.getPositionError();
    }
    return 0;
  }

  public void setOutputVoltage(double OutputVoltage) {
    m_controlMode = ControlMode.kOpenLoop;
    m_demand = OutputVoltage;
  }
 
  public void setPIDSetpoint(double angleDegrees) {
    m_controlMode = ControlMode.kPID;
    m_demand = angleDegrees;
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

  public Command upWardCommand(double OutputVoltage) {
    return openLoopCommand(()-> 2);
  }

  public Command downWarCommand(double OutputVoltage) {
    return openLoopCommand(()-> -2);
  }
  
                                                                    

  public Command goToAngleCommand(DoubleSupplier angleSupplier){
    return Commands.runEnd(
      () -> this.setPIDSetpoint(angleSupplier.getAsDouble()), this::stop, this);
  }

  public Command goToAngleCommand(double angleDegrees){
    return goToAngleCommand(()-> angleDegrees);
  }
}
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
import frc.robot.Constants.IntakePivotConstants;
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



public class IntakePivot extends SubsystemBase {
  private enum ControlMode {
    kOpenLoop, 
    kPID
  }

  private final WPI_TalonFX m_motor;

  private final CANCoder m_encoder;

  private ControlMode m_controlMode = ControlMode.kOpenLoop;
  private double m_demand;

  //
  // PID
  //

  private final PIDController m_pidController = new PIDController(IntakePivotConstants.kP, IntakePivotConstants.kI, IntakePivotConstants.kD);

  public IntakePivot() {
    m_motor = new WPI_TalonFX(Constants.IntakePivotConstants.MotorID, Constants.IntakePivotConstants.MotorCANBus);
    m_motor.configFactoryDefault();
    m_motor.setInverted(false);
    m_motor.setNeutralMode(NeutralMode.Brake);

    m_encoder = new CANCoder(Constants.IntakePivotConstants.EncoderID, Constants.IntakePivotConstants.EncoderCANBus);
    m_encoder.configFactoryDefault();

    m_motor.configForwardSoftLimitEnable(true);
    m_motor.configReverseSoftLimitEnable(true);
    m_motor.configReverseSoftLimitThreshold(IntakePivotConstants.reverseSoftLimit);
    m_motor.configForwardSoftLimitThreshold(IntakePivotConstants.forwardSoftLimit);
    m_motor.setNeutralMode(NeutralMode.Brake);


    m_encoder.configFactoryDefault();
    m_encoder.configSensorDirection(IntakePivotConstants.encoderDirection);
    m_encoder.configMagnetOffset(IntakePivotConstants.encoderOffset);
    m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    //Tells to use the non-motor encoder
    m_motor.configRemoteFeedbackFilter(m_encoder, 0);
    //Tells what encoder to use
    m_motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    //??
    m_motor.setSensorPhase(true);
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
        outputVoltage = m_pidController.calculate(getAngle(), m_demand);
        
        break;
      default:
        // What happened!?
        break;
    }


    // Do something with the motor    
    m_motor.setVoltage(outputVoltage);

    SmartDashboard.putNumber("Intake Pivot encoderAbosoluteAngle", m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Intake Pivot encoderAngle", m_encoder.getPosition());
    SmartDashboard.putNumber("Intake Pivot Motor Selected Sensor position", m_motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Intake Pivot Motor Error", getErrorAngle());
    SmartDashboard.putNumber("Intake Pivot Get Measurement", getAngle());
    SmartDashboard.putNumber("Intake Pivot Motor Output Voltage", outputVoltage);
    SmartDashboard.putNumber("Intake Pivot Motor Setpoint Position", m_demand);
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

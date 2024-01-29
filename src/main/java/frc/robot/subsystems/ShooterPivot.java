// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
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

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_sysIDAppliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
  private final MutableMeasure<Angle> m_sysIDAngle = MutableMeasure.mutable(Units.Degrees.of(0));
  private final MutableMeasure<Velocity<Angle>> m_sysIDAngluarVelocity = MutableMeasure.mutable(Units.DegreesPerSecond.of(0));

  //
  // PID
  //
  private final ArmFeedforward m_feedforward = new ArmFeedforward(0, 0.32, 3.59, 0.002); 
  private final ProfiledPIDController m_pidController = new ProfiledPIDController(
      ShooterPivotConstants.kP, 
      ShooterPivotConstants.kI, 
      ShooterPivotConstants.kD,
      new TrapezoidProfile.Constraints(Math.toRadians(90), Math.toRadians(90))
  );

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(
      // Tell SysId how to plumb the driving voltage to the motors.
      (Measure<Voltage> volts)-> setOutputVoltage(volts.in(Units.Volts)),
       // Tell SysId how to record a frame of data for each motor on the mechanism being
      // characterized.
      log->{
        log.motor("shooter-pivot")
          .voltage(m_sysIDAppliedVoltage)
          .angularPosition(m_sysIDAngle)
          .angularVelocity(m_sysIDAngluarVelocity);
      },
      this
    )
  );

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

    SmartDashboard.putNumber("SP kP", ShooterPivotConstants.kP);
    SmartDashboard.putNumber("SP kI", ShooterPivotConstants.kI);
    SmartDashboard.putNumber("SP kD", ShooterPivotConstants.kD);
    SmartDashboard.putNumber("SP kF", ShooterPivotConstants.kF);
  }

  public double getAngle() {
    return m_encoder.getAbsolutePosition();
  }


  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      m_encoder.setPositionToAbsolute();
    }

    // Determine output voltage
    double outputVoltage = 0;
    double currentAngleDegrees = getAngle();
    double currentAngleRadians = Math.toRadians(currentAngleDegrees);

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
        // double kF = SmartDashboard.getNumber("SP kF", ShooterPivotConstants.kF);

        // outputVoltage = kF * Math.cos(Math.toRadians(m_demand)) + m_pidController.calculate(currentAngle, m_demand);

        double pidOutputVoltage = m_pidController.calculate(currentAngleRadians, Math.toRadians(m_demand));
        double ffOutputVoltage = m_feedforward.calculate(m_pidController.getSetpoint().position, m_pidController.getSetpoint().velocity);
        
        outputVoltage = ffOutputVoltage + pidOutputVoltage;

        SmartDashboard.putNumber("shooter Pivot" + " PID Output Voltage", pidOutputVoltage);
        SmartDashboard.putNumber("shooter Pivot" + " FF Output Voltage", ffOutputVoltage);

        break;
      default:
        // What happened!?
        break;
    }


    // Do something with the motor    
    m_motor.setVoltage(outputVoltage);

    // SysID
    m_sysIDAppliedVoltage.mut_replace(outputVoltage, Units.Volts);
    m_sysIDAngle.mut_replace(currentAngleDegrees, Units.Degrees);
    m_sysIDAngluarVelocity.mut_replace(m_encoder.getVelocity(), Units.DegreesPerSecond);

    SmartDashboard.putNumber("Shooter Pivot" + " encoderAbosoluteAngle", m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Shooter Pivot" + " encoderAngle", currentAngleDegrees);
    SmartDashboard.putNumber("Shooter Pivot" + " Motor Selected Sensor position", m_motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shooter Pivot" + " Motor Error", getErrorAngle());
    // SmartDashboard.putNumber("Shooter Pivot" + " get measurement", currentAngleDegrees);
    SmartDashboard.putNumber("shooter Pivot" + " Motor Output Voltage", outputVoltage);
    SmartDashboard.putNumber("shooter Pivot" + " Motor Setpoint Position", m_demand);
  }

  public double getErrorAngle(){
    if (m_controlMode == ControlMode.kPID){
      return Math.toDegrees(m_pidController.getPositionError());
    }
    return 0;
  }

  public void setOutputVoltage(double OutputVoltage) {
    m_controlMode = ControlMode.kOpenLoop;
    m_demand = OutputVoltage;
  }
 
  public void setPIDSetpoint(double angleDegrees) {
    if (m_controlMode != ControlMode.kPID) {
      m_pidController.reset(Math.toRadians(getAngle()));
    }
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}

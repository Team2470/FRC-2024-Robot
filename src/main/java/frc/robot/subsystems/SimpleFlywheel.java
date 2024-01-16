// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FlyWheelConstants;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;



public class SimpleFlywheel extends SubsystemBase {
  private final CANSparkFlex m_leader;
  //private final CANSparkFlex m_follower;

  private final RelativeEncoder m_encoder;
  private final boolean m_isLeft;

  public SimpleFlywheel(int canID, boolean isLeft) {
    m_leader = new CANSparkFlex(canID, MotorType.kBrushless);
    m_leader.restoreFactoryDefaults();
    //m_follower = new CANSparkFlex(FlyWheelConstants.kFollowerID, MotorType.kBrushless);
    //m_follower.restoreFactoryDefaults();

    //m_follower.follow(m_leader, true);

    m_leader.setInverted(!isLeft);
    m_isLeft = isLeft;

    m_leader.setSmartCurrentLimit(40);
    //m_follower.setSmartCurrentLimit(40);

    // m_follower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    // m_follower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    // m_follower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    m_encoder = m_leader.getEncoder();
    m_leader.burnFlash();
    //m_follower.burnFlash();
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel " + (m_isLeft ? "Left" : "Right")+" Velocity", getVelocity());
  }

  public void setOutputVoltage(double OutputVoltage) {
    m_leader.setVoltage(OutputVoltage);
    SmartDashboard.putNumber("Flywheel "+ (m_isLeft ? "Left" : "Right") +  " output voltage", OutputVoltage);
  }
 
  public void stop() {
    setOutputVoltage(0);
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command spinCommand(DoubleSupplier OutputVoltageSupplier) {


    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return Commands.runEnd(
        () -> this.setOutputVoltage(OutputVoltageSupplier.getAsDouble()), this::stop, this);
        
  }

  public Command spinCommand(double OutputVoltage) {
    return spinCommand(()-> OutputVoltage);
  }
}

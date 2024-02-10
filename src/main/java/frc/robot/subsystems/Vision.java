// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Vision extends SubsystemBase {
  double RPM = 0;
  double Angle = 0;
  double distance = 0;
  /** Creates a new Vision. */
  public Vision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Vision RPM", getRPM());
    SmartDashboard.putNumber("Vision Angle", getAngle());

  }

  public double getDistance(){
  distance = SmartDashboard.getNumber("Select Distance", 0);
  return distance;
  }

  public double getRPM(){
    RPM = (227*(Math.pow(getDistance(), 0.578)));
    return RPM;
  }
  public double getAngle(){
    Angle = (1166*(Math.pow(getDistance(),-0.736)));
    return Angle;
  }

}

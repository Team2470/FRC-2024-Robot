// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class FlyWheelConstants {
    public static final int kRightID = 1;
    public static final int kLeftID = 2;

    public static final double kP = 0.001;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0.001155043488;

    public static double[][] kRPMValues = {
      {219, 6000},
      {169, 6000},
      {145, 4000},
      {121, 4000},
      {95, 2500},
      {71, 2500},
      {51.5, 2500},
  };

  public static double[][] kAngleValues = {
      {219, 22.06},
      {169, 27.3},
      {145, 30.23},
      {121, 33.57},
      {95, 45.35},
      {71, 50.18},
      {51.5, 60,38},
  };

    public static double getRPM(double distance){
      return (227*(Math.pow(distance, 0.578)));
    }

 
  }
  public static class ShooterPivotConstants {
    public static final int MotorID = 21;
    public static final int EncoderID = 21;
    public static final String MotorCANBus = "rio"; 
    public static final String EncoderCANBus = "rio";
    public static final int reverseSoftLimit = 50;
    public static final int forwardSoftLimit = 1024;
    public static final boolean encoderDirection = false;
    public static final double encoderOffset = 143.349609375+20+4.39453125+90-3;
;




    public static final double kP = 17.5;
    public static final double kI = 11;
    public static final double kD = 0.2;
    public static final double kF = 0;
    public static final double kG = 0.45;
    public static final double kV = 3.9;
    public static final double kA = 0.002;


    public static double getAngle(double distance) {
      return (1166*(Math.pow(distance,-0.736)))-2;
    } 


  }

  public static class IntakePivotConstants{
    public static final int MotorID = 3;
    public static final int EncoderID = 3;
    public static final String MotorCANBus = "rio"; 
    public static final String EncoderCANBus = "rio";
    public static final int reverseSoftLimit = 0;
    public static final int forwardSoftLimit = 0;
    public static final boolean encoderDirection = true;
    public static final int encoderOffset = 0;




    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0;
  }

}


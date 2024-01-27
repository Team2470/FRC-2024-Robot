// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  }
  public static class ShooterPivotConstants {
    public static final int MotorID = 21;
    public static final int EncoderID = 21;
    public static final String MotorCANBus = "rio"; 
    public static final String EncoderCANBus = "rio";
    public static final int reverseSoftLimit = 0;
    public static final int forwardSoftLimit = 1024;
    public static final boolean encoderDirection = false;
    public static final double encoderOffset = 144.31640625-1.669921875;
;




    public static final double kP = 0.001;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0.001155043488;

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


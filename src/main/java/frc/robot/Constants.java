// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants.ModuleConfig;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
public enum Robot {
	kCompetition,
	kSecond,
	kTestBed,
	kRyan2,
}

public enum CanBus {
	kRoboRIO("rio"),
	kCanivore("Canivore0");

	public final String bus_name;

	private CanBus(String value) {
	this.bus_name = value;
	}
}

public static Robot kRobot = Robot.kCompetition;

public static class CANdleConstants{
	public static final int CANdleID = 1;
	public static final int Intake_Index = 8;
	public static final int Intake_LEDnum = 8;
	public static final int TOF1_Index = 16;
	public static final int TOF1_LEDnum = 7;
	//  public static final int SpinningUp_Index = 20;
	//  public static final int SpinningUp_LEDnum = 6;
	//  public static final int FullSpeed_Index = 26;
	//  public static final int FullSpeed_LEDnum = 6;
	public static final int Vision_Index = 23;
	public static final int Vision_LEDNum = 7;
	public static final int Shooter_Index = 30;
	public static final int Shooter_LEDnum = 8;
}

public static class ClimberConstants{
	public static final int kLeftMotorID = 27;
	public static final int kLeftServoChannel = 9;
	public static final int kLeftRetractChannel = 8;
	public static final int kLeftExtendChannel = 9;
	public static final int kRightMotorID = 26;
	public static final int kRightServoChannel = 8;
	public static final int kRightRetractChannel = 7;
	public static final int kRightExtendChannel = 6;
}

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

	public static double getRPM(double distance) {
	if (kRobot == Robot.kSecond) {
		// Second robot
		return (227*(Math.pow(distance, 0.578)));
	}

	// Comp robot
	// return (178*(Math.pow(distance, 0.633)));
	// return (513*(Math.pow(distance, 0.439)));	
	if (distance < 150){
		return 4000;
	} else if (distance >= 150 && distance < 180){
		return 5000;
	} else if (distance >= 180 && distance < 250){
		return 6000;
	} else if(distance >= 250){
		return 5000;
	} else {
		return 4000;
	}


	}
}

public static class ShooterPivotConstants {
	public static final int MotorID = 21;
	public static final int EncoderID = 21;
	public static final String MotorCANBus = "rio";
	public static final String EncoderCANBus = "rio";
	public static int reverseSoftLimit = 250;
	public static int forwardSoftLimit = 1024;
	public static boolean encoderDirection = true;
	public static double encoderOffset = 140.889-21;

	// Fast gains
	// public static double kP = 70;
	// public static double kI = 11;
	// public static double kD = 1;

	public static double kP = 55;
	public static double kI = 5;
	public static double kD = 1;
	public static double kF = 0;
	public static double kG = 0.45;
	public static double kV = 3.9;
	public static double kA = 0.002;


	public static double getAngle(double distance) {
	if (kRobot == Robot.kSecond) {
		// Second robot
		// return (1166*(Math.pow(distance,-0.736)))-5;
		return (1764*(Math.pow(distance,-0.818)))-2;
	}

	// Comp robot
	// return (1848*(Math.pow(distance, -0.827)));
	// return (2250*(Math.pow(distance, -0.869))) + 2;
	return (1053*(Math.pow(distance, -0.708)))+2;
	}
}

public static class IntakeConstants{
	public static final int MotorID = 0;
}
public static class IntakePivotConstants{
	public static final int MotorID = 25;
	public static final int EncoderID = 22;
	public static final String MotorCANBus = "rio";
	public static final String EncoderCANBus = "rio";
	public static final int reverseSoftLimit = 0;
	public static final int forwardSoftLimit = 0;
	public static final boolean encoderDirection = false;
	public static final double encoderOffset = -133.5-119.44+60;

	public static final double kP = 0;
	public static final double kI = 0;
	public static final double kD = 0;
	public static final double kF = 0;
}

public static class DriveConstants {
	// : Physical constants (motors, sensors, ...)
	public static final double kDriveVoltageCompensation = 12;
	public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
	public static final double kWheelBaseLengthMeters = Units.inchesToMeters(11.375*2);
	public static final double kTrackWidthMeters = Units.inchesToMeters(12.375*2);

	public static final double kDriveGearReduction =
		SdsModuleConfigurations.MK4I_L3.getDriveReduction();

	public static final double kMaxDriveVelocityMetersPerSecond =
		DCMotor.getNEO(1).freeSpeedRadPerSec
			/ (2)
			* kDriveGearReduction
			* kWheelDiameterMeters
			* (kDriveVoltageCompensation / 12.0);

	public static final double kMaxAngularVelocityRadiansPerSecond =
		kMaxDriveVelocityMetersPerSecond
			/ Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseLengthMeters / 2.0);

	public static final SwerveDriveKinematics kDriveKinematics =
		new SwerveDriveKinematics(
			new Translation2d(kWheelBaseLengthMeters / 2, kTrackWidthMeters / 2),
			new Translation2d(kWheelBaseLengthMeters / 2, -kTrackWidthMeters / 2),
			new Translation2d(-kWheelBaseLengthMeters / 2, kTrackWidthMeters / 2),
			new Translation2d(-kWheelBaseLengthMeters / 2, -kTrackWidthMeters / 2));
	// : IMU constants
	public static final int kPigeonID = 0;
	public static final CanBus kPigeonCANBus = CanBus.kRoboRIO;

	public static class ModuleConfig {
	public int line, col;
	public int encoderID, steeringID, drivingID;
	public String name;
	public Rotation2d offset;

	public ModuleConfig(String name) {
		this.name = name;
	}

	public ModuleConfig setTab(int line, int col) {
		this.line = line;
		this.col = col;
		return this;
	}

	public ModuleConfig setSteeringID(int id) {
		this.steeringID = id;
		return this;
	}

	public ModuleConfig setEncoderID(int id) {
		this.encoderID = id;
		return this;
	}

	public ModuleConfig setDrivingID(int id) {
		this.drivingID = id;
		return this;
	}

	public ModuleConfig setOffset(double angle) {
		this.offset = Rotation2d.fromDegrees(angle);
		return this;
	}
	}
	// : specific module config
	// When calibrating the bevel gears should face to the left

	public static ModuleConfig kFrontLeft =
		new ModuleConfig("Front Left")
			.setDrivingID(13)
			.setEncoderID(13)
			.setSteeringID(13)
			.setOffset(-299.04+180)
			.setTab(0, 0);

	public static ModuleConfig kFrontRight =
		new ModuleConfig("Front Right")
			.setDrivingID(14)
			.setEncoderID(14)
			.setSteeringID(14)
			.setOffset(-186.328+180)
			.setTab(0, 2);

	public static ModuleConfig kBackRight =
		new ModuleConfig("Back Right")
			.setDrivingID(11)
			.setEncoderID(11)
			.setSteeringID(11)
			.setOffset(-48.076+180)
			.setTab(0, 6);

	public static ModuleConfig kBackLeft =
		new ModuleConfig("Back Left")
			.setDrivingID(12)
			.setEncoderID(12)
			.setSteeringID(12)
			.setOffset(-105.029-0.088+180)
			.setTab(0, 4);
}

public static class AutoConstants {
	public static final double kAutoVoltageCompensation = 10;

	public static final PIDConstants kPIDTranslation =
		new PIDConstants(5.0, 0, 0); // : PID constants for translation error
	public static final PIDConstants kPIDRotation =
		new PIDConstants(2.0, 0, 0); // : Theta rotation,

	public static final PathConstraints kPathConstraints =
	new PathConstraints(3, 2, Constants.DriveConstants.kMaxAngularVelocityRadiansPerSecond,
								Constants.DriveConstants.kMaxAngularVelocityRadiansPerSecond);
}

public static class VisionConstants{
	public static Transform3d kFrontRightCamera = new Transform3d(
	new Translation3d(Units.inchesToMeters(-5.1), Units.inchesToMeters(14.391), Units.inchesToMeters(6.626)),
	new Rotation3d(0, Units.degreesToRadians(25.0), 0)
	);
}

// public static class ClimberConstants{
//   public static final int kLeftMotorID = 25;
//   public static final int kLeftServoChannel = 1;
//   public static final int kRightMotorID = 26;
//   public static final int kRightServoChannel = 2;
// }
public static void override() {
	String serialNumber = RobotController.getSerialNumber();
	System.out.println("roboRIO Serial: "+serialNumber);

	switch (serialNumber) {
	case "03060ff5":
	kRobot = Robot.kSecond;

	DriveConstants.kFrontLeft = new ModuleConfig("Front Left")
			.setDrivingID(16)
			.setEncoderID(16)
			.setSteeringID(16)
			.setOffset(-141.85548853232115+180)
			.setTab(0, 0);

	DriveConstants.kFrontRight = new ModuleConfig("Front Right")
			.setDrivingID(14)
			.setEncoderID(14)
			.setSteeringID(14)
			.setOffset(-126.12304687500001+180)
			.setTab(0, 2);

	DriveConstants.kBackLeft = new ModuleConfig("Back Left")
			.setDrivingID(12)
			.setEncoderID(12)
			.setSteeringID(12)
			.setOffset(-321.50391638394024+180)
			.setTab(0, 4);

	DriveConstants.kBackRight = new ModuleConfig("Back Right")
			.setDrivingID(10)
			.setEncoderID(10)
			.setSteeringID(10)
			.setOffset(-210.234375+180)
			.setTab(0, 6);

	ShooterPivotConstants.kP = 17.5;
	ShooterPivotConstants.kI = 11;
	ShooterPivotConstants.kD = 0.2;
	ShooterPivotConstants.kF = 0;
	ShooterPivotConstants.kG = 0.45;
	ShooterPivotConstants.kV = 3.9;
	ShooterPivotConstants.kA = 0.002;

	ShooterPivotConstants.encoderOffset = 143.349609375+20+4.39453125+90-3;
	ShooterPivotConstants.encoderDirection = false;
	ShooterPivotConstants.reverseSoftLimit = 50;
	ShooterPivotConstants.forwardSoftLimit = 1024;

	VisionConstants.kFrontRightCamera = new Transform3d(
		new Translation3d(Units.inchesToMeters(-5.1), Units.inchesToMeters(14.391), Units.inchesToMeters(6.626)),
		new Rotation3d(0, Units.degreesToRadians(25.0), 0)
	);

	break;
	case "0324152A":
	kRobot = Robot.kRyan2;
	break;
	default:
	// Do nothing
	}
}
}

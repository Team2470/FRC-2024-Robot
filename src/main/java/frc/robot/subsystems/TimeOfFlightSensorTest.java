package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class TimeOfFlightSensorTest extends SubsystemBase {
	private final TimeOfFlight m_TimeOfFlight_1;
	// private final TimeOfFlight m_TimeOfFlight_2;

	// private double initialEncoderValue;
	private int Tof;
	public TimeOfFlightSensorTest(int TofID){
		m_TimeOfFlight_1 = new TimeOfFlight(TofID);
		// m_TimeOfFlight_2 = new TimeOfFlight(2);
		Tof = TofID;
	}

	public double getRange_TOF1(){
		return m_TimeOfFlight_1.getRange();
	}

	public double getAmbientLightLevel_TOF1(){
		return m_TimeOfFlight_1.getAmbientLightLevel();
	}

	public double getRangeSigma_TOF1(){
		return m_TimeOfFlight_1.getRangeSigma();
	}

	public double getSampleTime_TOF1(){
		return m_TimeOfFlight_1.getSampleTime();
	}

	public TimeOfFlight.RangingMode getRangingMode_TOF1() {
		return m_TimeOfFlight_1.getRangingMode();
	}

	// public double getRange_TOF2(){
	//      return m_TimeOfFlight_2.getRange();
	//  }

	// public double getAmbientLightLevel_TOF2(){
	//     return m_TimeOfFlight_2.getAmbientLightLevel();
	// }

	// public double getRangeSigma_TOF2(){
	//     return m_TimeOfFlight_2.getRangeSigma();
	// }

	// public double getSampleTime_TOF2(){
	//     return m_TimeOfFlight_2.getSampleTime();
	// }

	// public TimeOfFlight.RangingMode getRangingMode_TOF2() {
	//     return m_TimeOfFlight_2.getRangingMode();
	// }

	public boolean isTOF1WithinRange() {
		return (this.getRange_TOF1() < 175);

	}
	public boolean isTOF2WithinRange() {
		return (this.getRange_TOF1() < 200);

	}

	public boolean isTOF1OutOfRange(){
		return (this.getRange_TOF1() > 300);
	}
	// public boolean isTOF2WithinRange() {
	//     return (this.getRange_TOF2() < 200);

	// }

	public double variableVoltage(){
		double range = this.getRange_TOF1();
		double volts;
		if (range > 0 && range <= 200){
			volts = range / 40;
		}
		if (range > 200) {
			volts = 5;
		}
		else {
			volts = range / 40;
		}
		return volts;
	}

	public Command waitUntilTOF1InRange(){
		return Commands.waitUntil(()-> this.isTOF1WithinRange());
	}

	// public Command waitUntilTOF2InRange(){
	//     return Commands.waitUntil(()-> this.isTOF2WithinRange());
	// }

	public Command waitUntilOutOfRange(){
		return Commands.waitUntil(()-> !this.isTOF1WithinRange());
	}

	public Command wait1SecondsCommand(){
		return Commands.waitSeconds(1);
	}

	public Command wait10SecondsCommand(){
		return Commands.waitSeconds(10);
	}

	public Command variableVoltageTest(SimpleShooterFeeder m_feeder) {
		return Commands.repeatingSequence(
			this.waitUntilOutOfRange(),
			m_feeder.forward(() ->variableVoltage()).until(()-> !this.isTOF1WithinRange())
		);
	}

	public Command feedBack5Rotations(SimpleShooterFeeder m_feeder) {
		return Commands.repeatingSequence(
			m_feeder.resetEncoder(),
			m_feeder.reverse().until(()-> m_feeder.isEncoderPast5Rotations()),
			this.wait1SecondsCommand()
		);
	}


	public Command feederIntakeCommand(SimpleShooterFeeder m_feeder) {
		return Commands.repeatingSequence(
			m_feeder.reverse().until(()->this.isTOF1WithinRange()),
			this.wait10SecondsCommand()
		);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Range_TOF" + Tof, getRange_TOF1());
		SmartDashboard.putNumber("Ambient Light_TOF" + Tof, getRange_TOF1());
		SmartDashboard.putNumber("Range Sigma_TOF" + Tof, getRange_TOF1());
		SmartDashboard.putNumber("Sample Time_TOF" + Tof, getRange_TOF1());
		SmartDashboard.putBoolean("Is TOF" + Tof + "Within Range",isTOF1WithinRange());
		// SmartDashboard.putNumber("Range_TOF2", getRange_TOF2());
		// SmartDashboard.putBoolean("Is TOF2 Within Range",isTOF2WithinRange());
		SmartDashboard.putNumber("Variable Voltage TOF" + Tof, variableVoltage());
	}




}

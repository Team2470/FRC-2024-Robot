package frc.robot.subsystems;

import com.fasterxml.jackson.core.StreamWriteCapability;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SimpleShooterFeeder;

public class TimeOfFlightSensorTest extends SubsystemBase {
    private final TimeOfFlight m_TimeOfFlight_1;
    //private final TimeOfFlight m_TimeOfFlight_2;

    private double initialEncoderValue;

    public TimeOfFlightSensorTest(){
        m_TimeOfFlight_1 = new TimeOfFlight(1);
        //m_TimeOfFlight_2 = new TimeOfFlight(2);
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
    //     return m_TimeOfFlight_2.getRange();
    // }

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
        return (this.getRange_TOF1() > 200 && this.getRange_TOF1() < 400);

    }

    public Command waitUntilInRange(){
        return Commands.waitUntil(()-> this.isTOF1WithinRange());
    }

    public Command waitUntilOutOfRange(){
        return Commands.waitUntil(()-> !this.isTOF1WithinRange());
    }

    public Command wait5SecondsCommand(){
        return Commands.waitSeconds(5);
    }

    public Command sequenceTest(SimpleShooterFeeder m_feeder) {
        return Commands.repeatingSequence(
            //wait5SecondsCommand(),
            this.waitUntilOutOfRange(),
            m_feeder.SimpleShooterFeeder_forwardsCommand().until(()->!this.isTOF1WithinRange())//,
            /*m_feeder.SimpleShooterFeeder_stopCommand()*/
        );
    }

    public boolean isEncoderWithin10(SimpleShooterFeeder m_feeder) {
        return (this.initialEncoderValue - m_feeder.getEncoderPosition() > 10);
    }

    public void readInitialEncoderValue(SimpleShooterFeeder m_feeder) {
        this.initialEncoderValue = m_feeder.getEncoderPosition();
    }

    // public Command encoderRotationTest(SimpleShooterFeeder m_feeder) {
    //     return Commands.repeatingSequence(
    //         this.readInitialEncoderValue(m_feeder),
    //         m_feeder.SimpleShooterFeeder_forwardsCommand().until(()->!this.isEncoderPastT10Rotations())//,
    //         /*m_feeder.SimpleShooterFeeder_stopCommand()*/
    //     );
    // }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Range_TOF1", getRange_TOF1());
        SmartDashboard.putNumber("Ambient Light_TOF1", getRange_TOF1());
        SmartDashboard.putNumber("Range Sigma_TOF1", getRange_TOF1());
        SmartDashboard.putNumber("Sample Time_TOF1", getRange_TOF1());
        SmartDashboard.putBoolean("Is Within Range",isTOF1WithinRange());
        //SmartDashboard.putData("Ranging Mode Time_TOF1", getRange_TOF1());
        // SmartDashboard.putNumber("Range_TOF2", getRange_TOF2());
        // SmartDashboard.putNumber("Ambient Light_TOF2", getRange_TOF2());
        // SmartDashboard.putNumber("Range Sigma_TOF2", getRange_TOF2());
        // SmartDashboard.putNumber("Sample Time_TOF2", getRange_TOF2());
        //SmartDashboard.putData("Ranging Mode_TOF2", getRange_TOF2());
    }




}
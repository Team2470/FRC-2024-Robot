package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.RelativeEncoder;

import frc.robot.subsystems.IntakeSubsystem;
public class TimeOfFlightSensorTest extends SubsystemBase{
    private final TimeOfFlight m_TimeOfFlight_1;
    private final TimeOfFlight m_TimeOfFlight_2;


    private double initialEncoderValue;
    public TimeOfFlightSensorTest () {
        m_TimeOfFlight_1 = new TimeOfFlight(1);
        m_TimeOfFlight_2 = new TimeOfFlight(2);

    }
    public double getRange_TOF1(){
        return m_TimeOfFlight_1.getRange();
    }
        public double getRange_TOF2(){
        return m_TimeOfFlight_2.getRange();
    }
    public double getAmbientLightLevel_TOF1() {
        return m_TimeOfFlight_1.getAmbientLightLevel();
    }
    public double  getRangeSigma_TOF1() {
            return m_TimeOfFlight_1.getRangeSigma();
    }
    public double  getSampleTime_TOF1() {
            return m_TimeOfFlight_1.getSampleTime();
    }
    public boolean  isTOF1_withinRange() {
        return (this.getRange_TOF1() < 200 && this.getRange_TOF1() > 0);
    }
    public boolean  isTOF2_withinRange() {
        return (this.getRange_TOF2() < 200 && this.getRange_TOF2() > 0);
    }
    public Command waitUntilTOF1InRange() {
        return Commands.waitUntil(() -> this.isTOF1_withinRange());
    }
    public Command waitUntilTOF2InRange() {
        return Commands.waitUntil(() -> this.isTOF2_withinRange());
    }
    public Command waitUntilOutOfRange() {
        return Commands.waitUntil(() -> !this.isTOF1_withinRange());
    }
    public Command wait5SecndsCommand(){
        return Commands.waitSeconds(5);
    }
    public Command sequenceTest(IntakeSubsystem m_intake) {
        return Commands.repeatingSequence(
            this.waitUntilTOF1InRange(), 
            m_intake.test_forwardsCommand().until(() -> this.isTOF2_withinRange()),
            this.wait1secondCommand(),
            feedBack5Rotations(m_intake)
            );
    }
    public Command wait1secondCommand() {
        return Commands.waitSeconds(1);
    }
    public Command feedBack5Rotations(IntakeSubsystem m_intake) {
        return Commands.repeatingSequence(
            m_intake.zeroEncoderCommand(),
            m_intake.test_reverseCommand().until(() -> m_intake.isEncoderPast5Rotation()));
    }

    public void readInitialEncoderValue(IntakeSubsystem m_encoder) {
        this.initialEncoderValue = m_encoder.getEncoderPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Range TOF1", getRange_TOF1());
        SmartDashboard.putNumber("Ambient Light TOF1", getAmbientLightLevel_TOF1());
        SmartDashboard.putNumber("Range Sigma TOF1",  getRangeSigma_TOF1());
        SmartDashboard.putNumber("Sample Time TOF1", getSampleTime_TOF1());
        SmartDashboard.putBoolean("Is TOF1 within range", isTOF1_withinRange());

        SmartDashboard.putNumber("Range TOF2", getRange_TOF2());
        SmartDashboard.putBoolean("Is TOF2 within range", isTOF2_withinRange());
}

        
}







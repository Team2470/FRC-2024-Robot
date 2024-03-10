package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants;
import frc.robot.Constants.CANdleConstants;



import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;


public class LEDSubsystem extends SubsystemBase {

        private Animation m_toAnimate = null;
        private final CANdle m_candle = new CANdle(CANdleConstants.CANdleID, "rio");
        private final int LedCount = 38;
        // private final int LedCount = 8;
        private final int numSegments = 4;
        private CommandXboxController joystick;

        private int[] m_LEDs = new int[numSegments];
        private int activeSegment = 0;

        public LEDSubsystem(CommandXboxController joy) {   
            this.joystick = joy;
            CANdleConfiguration config_candle = new CANdleConfiguration();
            config_candle.statusLedOffWhenActive = true;
            config_candle.disableWhenLOS = false;
            config_candle.brightnessScalar = 0.5;
            config_candle.stripType = LEDStripType.RGB;
            config_candle.vBatOutputMode = VBatOutputMode.Modulated;
            m_candle.configAllSettings(config_candle,100);

            initDisabledLEDs();

            m_candle.setLEDs(0, 0, 0,0, 0, LedCount);
            // m_candle.setLEDs(254, 1, 154, 0, 0, 8);

            for(LEDStripSegments segment : LEDStripSegments.values()) {
                m_LEDs[segment.value] = LEDStripStatus.SegmentRed.value;
            }

        }
    



    public enum LEDStripSegments {
        Intake(0),
        TOF1(1),
        Photon(2),
        Shooter(3);
        
        
        // SpinningUp(2),
        // FullSpeed(3),
        // Shooting(4),


  
        public final int value;
        private LEDStripSegments(int value) {
            this.value = value;
        }
    }


    public enum LEDStripStatus {
        SegmentOff(0),
        SegmentsGreen(1),
        SegmentRed(2),
        SegmentPink (3),
        SegmentYellow(4);

        // AllGreen(3),
        // AllRed(4),
        // AllOff(5),
        // AllPink(6);
        public final int value;
        private LEDStripStatus(int value) {
            this.value = value;
        }

    }

    public void setLEDStatus() {
        int status = 0;
        int index = 0;
        int LEDnum = 0;
        
        for (LEDStripSegments segment : LEDStripSegments.values()) {
            status = m_LEDs[segment.value];

            if (segment.value == LEDStripSegments.Intake.value) {
                index = CANdleConstants.Intake_Index;
                LEDnum = CANdleConstants.Intake_LEDnum;
            } else if (segment.value == LEDStripSegments.TOF1.value) {
                index = CANdleConstants.TOF1_Index;
                LEDnum = CANdleConstants.TOF1_LEDnum;
            // } else if (segment.value == LEDStripSegments.SpinningUp.value) {
            //     index = CANdleConstants.SpinningUp_Index;
            //     LEDnum = CANdleConstants.SpinningUp_LEDnum;
            // } else if (segment.value == LEDStripSegments.FullSpeed.value) {
            //     index = CANdleConstants.FullSpeed_Index;
            //     LEDnum = CANdleConstants.FullSpeed_LEDnum;
            } else if (segment.value == LEDStripSegments.Shooter.value) {
                index = CANdleConstants.Shooter_Index;
                LEDnum = CANdleConstants.Shooter_LEDnum;
            }

            switch (status) {
                case 0: // Segment OFF
                m_candle.setLEDs(0, 0, 0,0,index,LEDnum); break;
                case 1: // Segment GREEN
                m_candle.setLEDs(0, 255, 0,0,index,LEDnum); break;
                case 2: // Segment RED
                m_candle.setLEDs(255, 0, 0,0,index,LEDnum); break;
                case 3: // Segment PINK
                m_candle.setLEDs(254, 1, 154,0 ,index,LEDnum); break;
                case 4: // Segment YELLOW
                m_candle.setLEDs(255, 255, 0, 0, index, LEDnum); break;
            }
        }

        
    }
    public void changeLEDStatus(int segment, int status) {
        m_LEDs[segment] = status;
    }

    public void changeIntakeGreen() {
        changeLEDStatus(LEDStripSegments.Intake.value, LEDStripStatus.SegmentsGreen.value);
    }
    public void changeIntakeRed() {
        changeLEDStatus(LEDStripSegments.Intake.value, LEDStripStatus.SegmentRed.value);
    }
    public void changeIntakePink() {
        changeLEDStatus(LEDStripSegments.Intake.value, LEDStripStatus.SegmentPink.value);
    }

    public void changeTOF1Green() {
        changeLEDStatus(LEDStripSegments.TOF1.value, LEDStripStatus.SegmentsGreen.value);
    }
    public void changeTOF1Red() {
        changeLEDStatus(LEDStripSegments.TOF1.value, LEDStripStatus.SegmentRed.value);
    }
    public void changeTOF1Pink() {
        changeLEDStatus(LEDStripSegments.TOF1.value, LEDStripStatus.SegmentPink.value);
    }

    public void changeVisionGreen() {
        changeLEDStatus(LEDStripSegments.Photon.value, LEDStripStatus.SegmentsGreen.value);
    }
      public void changeVisionRed() {
        changeLEDStatus(LEDStripSegments.Photon.value, LEDStripStatus.SegmentRed.value);
    }
      public void changeVisionPink() {
        changeLEDStatus(LEDStripSegments.Photon.value, LEDStripStatus.SegmentPink.value);
    }

    public void changeShooterGreen() {
        changeLEDStatus(LEDStripSegments.Shooter.value, LEDStripStatus.SegmentsGreen.value);
    }
    public void changeShooterRed() {
        changeLEDStatus(LEDStripSegments.Shooter.value, LEDStripStatus.SegmentRed.value);
    }
    public void changeShooterPink() {
        changeLEDStatus(LEDStripSegments.Shooter.value, LEDStripStatus.SegmentPink.value);
    }
    public void changeShooterYellow() {
        changeLEDStatus(LEDStripSegments.Shooter.value, LEDStripStatus.SegmentYellow.value);
    }
    public int getLEDStatus(int segment){
        return m_LEDs[segment];
    }

    public void changeActiveSegment(int segment) {
        activeSegment = segment;
    }

    public int getActiveSegment() {
        return activeSegment;
    }

    public void incrementSegment(int segment) {
        switch(segment) {
            case 0: changeActiveSegment(1); break;
            case 1: changeActiveSegment(2); break;
            case 2: changeActiveSegment(3); break;
            case 3: changeActiveSegment(0); break;
            // case 3: changeActiveSegment(4); break;
            // case 4: changeActiveSegment(0); break;
        }
    }
    public void incrementStatus(int segment) {
        switch(m_LEDs[segment]) {
            case 0: changeLEDStatus(segment, 1); break;
            case 1: changeLEDStatus(segment, 2); break;
            case 2: changeLEDStatus(segment, 3); break;
            case 3: changeLEDStatus(segment, 0); break;
        }
    }

    public void initDisabledLEDs() {
        m_toAnimate = new SingleFadeAnimation(254,1,154,0,0.5,LedCount);
    }

    public void animateDisabledLEDs(){
        m_candle.animate(m_toAnimate,0);
    }

    public void stopAnimatingLEDs(){
        m_candle.clearAnimation(0);
    }

    @Override   
    public void periodic() {
        //setLEDStatus();

        if (DriverStation.isEnabled()){
            stopAnimatingLEDs();
            setLEDStatus();
        }
        else {
            animateDisabledLEDs();
        }

        SmartDashboard.putNumber("Intake Segment Status", getLEDStatus(LEDStripSegments.Intake.value));
        SmartDashboard.putNumber("TOF Segment Status", getLEDStatus(LEDStripSegments.TOF1.value));
        // SmartDashboard.putNumber("Spinning Up Segment Status", getLEDStatus(LEDStripSegments.SpinningUp.value));
        // SmartDashboard.putNumber("Full Speed Segment Status", getLEDStatus(LEDStripSegments.FullSpeed.value));
        SmartDashboard.putNumber("Shooting Segment Status", getLEDStatus(LEDStripSegments.Shooter.value));
        SmartDashboard.putNumber("Photon Vision Status", getLEDStatus(LEDStripSegments.Photon.value));

    //    m_candle.setLEDs(255, 0, 0, 0, 0, 8);
    //    m_candle.setLEDs(0, 255, 0, 0, 8, 5);
    //    m_candle.setLEDs(0, 0, 255, 0, 13, 5);
    }

    public Command incrementSegmentCommand() {
        return Commands.runOnce(() -> incrementSegment(activeSegment), this);
    }
    public Command incrementStatusCommand() {
        return Commands.runOnce(() -> incrementStatus(activeSegment), this);
    }

    
}
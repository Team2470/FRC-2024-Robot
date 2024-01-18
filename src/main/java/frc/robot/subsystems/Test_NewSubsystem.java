package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Test_NewSubsystem extends SubsystemBase {
    private final CANSparkFlex m_test;
    private final RelativeEncoder m_encoder;

    public Test_NewSubsystem(int canID) {
    
        m_test = new CANSparkFlex(canID, MotorType.kBrushless);
        m_test.restoreFactoryDefaults();

        m_test.setInverted(false);
        m_test.setSmartCurrentLimit(40);

        m_encoder = m_test.getEncoder();
        m_test.burnFlash();

    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Test Velocity", getVelocity());
    }

    public void test_forwards() {
        m_test.setVoltage(5);
    }

    public void test_reverse() {
        m_test.setVoltage(-5);
    }

    public void stop() {
        m_test.stopMotor();
    }
    

    public Command test_forwardsCommand() {
        return Commands.runEnd(
            ()-> this.test_forwards(),
             this::stop,
             this );
    }

    public Command test_reverseCommand() {
        return Commands.runEnd(
            ()-> this.test_reverse(),
             this::stop,
             this );
    }

}
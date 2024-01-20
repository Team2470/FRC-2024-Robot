package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_intake; 
    private final RelativeEncoder m_encoder;
    public IntakeSubsystem(int canID) {
        m_intake = new CANSparkMax(canID, MotorType.kBrushless);

        m_intake.restoreFactoryDefaults();
        m_intake.setInverted(false);
        m_intake.setSmartCurrentLimit(40);

        m_encoder = m_intake.getEncoder();
        m_intake.burnFlash();       

    }
    public double getVelocity(){
        return m_encoder.getVelocity();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Velocity", getVelocity());

  }
  public void intake() {
    m_intake.setVoltage(5);
  }
  public void reverse_intake () {
    m_intake.setVoltage(-5);

  }
  public void stop() {
    m_intake.stopMotor();
  }
  public Command test_forwardsCommand() {
    return Commands.runEnd(
        ()-> this.intake(),
        this::stop,
        this);
  }
    public Command test_reverseCommand() {
        return Commands.runEnd(
        ()-> this.reverse_intake(),
        this::stop,
        this);
  }
}

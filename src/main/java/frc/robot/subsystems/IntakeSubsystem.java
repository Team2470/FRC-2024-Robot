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
    public final RelativeEncoder m_encoder;
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
  public double getEncoderPosition() {
    return m_encoder.getPosition();
  }
    public double getEncoderCPR() {
    return m_encoder.getCountsPerRevolution();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Velocity", getVelocity());
    SmartDashboard.putNumber("Encoder counts per rev", getEncoderCPR());
    SmartDashboard.putNumber("Encoder Pos", getEncoderPosition());
    SmartDashboard.putBoolean("is encoder past 5 rotations", isEncoderPast5Rotation());
  }
  public void intake() {
    m_intake.setVoltage(1);
  }
  public void reverse_intake () {
    m_intake.setVoltage(-1);

  }
  public void forwardsVariableVoltage(double volts) {
    m_intake.setVoltage(volts);
  }
  public void reverseVariableVoltage (double volts) {
    m_intake.setVoltage(volts);

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
    public Command reverseVariableCommand(double volts) {
        return Commands.runEnd(
        ()-> this.reverseVariableVoltage(volts),
        this::stop,
        this);
  }
    public Command forwardsVariableCommand(double volts) {
    return Commands.runEnd(
        ()-> this.forwardsVariableVoltage(volts),
        this::stop,
        this);
  }
    public Command test_reverseCommand() {
        return Commands.runEnd(
        ()-> this.reverse_intake(),
        this::stop,
        this);
  }

    public Command test_stopCommand() {
        return Commands.runEnd(
        ()-> this.stop(),
        this::stop,
        this);
      }
    public void zeroEncoderValue() {
        m_encoder.setPosition(0);
      }      
    public Command zeroEncoderCommand() {
        return Commands.runOnce(() -> this.zeroEncoderValue(), this);
    }
    public boolean isEncoderPast5Rotation () {
      return (getEncoderPosition() > 5 || getEncoderPosition() < -5 );
  }

}


package frc.robot.subsystems; 
 
import edu.wpi.first.wpilibj2.command.Command; 
import edu.wpi.first.wpilibj2.command.Commands; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
 
import com.revrobotics.CANSparkMax; 
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.CANSparkLowLevel.MotorType; 
 
public class Intake extends SubsystemBase { 
    private final PWMSparkMax m_intake;  
    public Intake() { 
        m_intake = new PWMSparkMax(IntakeConstants.MotorID); 
  
        m_intake.setInverted(false);       
 
    } 
  @Override 
  public void periodic() { 
 
  } 
  public void intake() { 
    m_intake.setVoltage(12); 
  } 
  public void reverse_intake () { 
    m_intake.setVoltage(-12); 
 
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

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ResourceBundle.Control;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SimpleShooterFeeder extends SubsystemBase {
    private final CANSparkMax m_SimpleShooterFeeder;
    private final RelativeEncoder m_Encoder;

    public SimpleShooterFeeder(int canID) {

        m_SimpleShooterFeeder = new CANSparkMax(canID, MotorType.kBrushless);
        m_SimpleShooterFeeder.restoreFactoryDefaults();
        m_SimpleShooterFeeder.setInverted(false);
        m_SimpleShooterFeeder.setSmartCurrentLimit(40);
        m_Encoder = m_SimpleShooterFeeder.getEncoder();
        m_SimpleShooterFeeder.burnFlash();



    }

    public double getVelocity() {
        return m_Encoder.getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder Velocity", getVelocity());
    }


    public void feedShooter() {
        m_SimpleShooterFeeder.setVoltage(5);
    }
    public void reverseFeeder() {
        m_SimpleShooterFeeder.setVoltage((-5));
    }

    public void stop() {
        m_SimpleShooterFeeder.stopMotor();

    }


    public Command SimpleShooterFeeder_forwardsCommand() {
        return Commands.runEnd(
            ()-> this.feedShooter(),
            this::stop,
            this);
    }


    public Command SimpleShooterFeeder_reverseCommand() {
        return Commands.runEnd(
            ()-> this.reverseFeeder(),
            this::stop,
            this);
    }



}
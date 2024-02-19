package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SimpleShooterFeeder extends SubsystemBase {
    private final CANSparkMax m_SimpleShooterFeeder;
    // public final RelativeEncoder m_encoder;

    public SimpleShooterFeeder(int canID) {
        m_SimpleShooterFeeder = new CANSparkMax(canID, MotorType.kBrushed);
        m_SimpleShooterFeeder.restoreFactoryDefaults();
        m_SimpleShooterFeeder.setInverted(false);
        m_SimpleShooterFeeder.setSmartCurrentLimit(40);
        // m_encoder = m_SimpleShooterFeeder.getEncoder();
        m_SimpleShooterFeeder.burnFlash();
    }

    public double getVelocity() {
        // return m_encoder.getVelocity();
        return 0;
    }
    public double getEncoderPosition() {
        // return m_encoder.getPosition();
        return 0;
    }

    public double getEncoderCPR() {
        // return m_encoder.getCountsPerRevolution();
        return 0;
    }

    public boolean isEncoderPast5Rotations() {
        return (getEncoderPosition() > 5 || getEncoderPosition() < -5);
    }
    public void zeroEncoderValue() {
        m_encoder.setPosition(0);
    }




    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder Velocity", getVelocity());
        SmartDashboard.putNumber("Encoder Position", getEncoderPosition());
        SmartDashboard.putNumber("Encoder Counts Per Revolution", getEncoderCPR());
        SmartDashboard.putBoolean("Is Encoder Past 5 Rotations", isEncoderPast5Rotations());
    }


    public void feedShooter() {
        m_SimpleShooterFeeder.setVoltage(12);
    }
    public void reverseFeeder() {
        m_SimpleShooterFeeder.setVoltage((-12));
    }

    public void stopFeeder() {
        m_SimpleShooterFeeder.stopMotor();

    }

    public void feederForwards(double volts) {
        m_SimpleShooterFeeder.setVoltage(volts);
    }
    public void feederReverse(double volts) {
        m_SimpleShooterFeeder.setVoltage(volts);
    }


    public Command forward() {
        return Commands.runEnd(
            () -> this.feedShooter(),
            this::stopFeeder,
            this);
    }


    public Command reverse() {
        return Commands.runEnd(
            ()-> this.reverseFeeder(),
            this::stopFeeder,
            this);
    }

    public Command stop() {
        return Commands.runEnd(() -> this.stopFeeder(), this::stopFeeder, this);
    }

    public Command resetEncoder() {
        return Commands.runOnce(() -> this.zeroEncoderValue(), this);
    }

    public Command forward(DoubleSupplier volts) {
        return Commands.runEnd(
            ()-> this.feederForwards(volts.getAsDouble()),
            this::stopFeeder,
            this);
    }

    public Command reverse(DoubleSupplier volts) {
        return Commands.runEnd(
            ()-> this.feederReverse(volts.getAsDouble()),
            this::stopFeeder,
            this);
    }
}
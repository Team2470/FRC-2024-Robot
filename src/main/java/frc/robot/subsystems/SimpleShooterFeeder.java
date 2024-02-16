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
    public final RelativeEncoder m_encoder;

    public SimpleShooterFeeder(int canID) {

        m_SimpleShooterFeeder = new CANSparkMax(canID, MotorType.kBrushed);
        m_SimpleShooterFeeder.restoreFactoryDefaults();
        m_SimpleShooterFeeder.setInverted(false);
        m_SimpleShooterFeeder.setSmartCurrentLimit(40);
        m_encoder = m_SimpleShooterFeeder.getEncoder();
        m_SimpleShooterFeeder.burnFlash();



    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }
    public double getEncoderPosition() {
        return m_encoder.getPosition();
    }

    public double getEncoderCPR() {
        return m_encoder.getCountsPerRevolution();
    }

    public boolean isEncoderPast5Rotations( ) {
        return (getEncoderPosition() > 5 || getEncoderPosition() < -5);
    }


    public void zeroEncoderValue(){
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
        m_SimpleShooterFeeder.setVoltage(1);
    }
    public void reverseFeeder() {
        m_SimpleShooterFeeder.setVoltage((-1));
    }

    public void stop() {
        m_SimpleShooterFeeder.stopMotor();

    }

    public void forwardsVariableVoltage(double volts) {
        m_SimpleShooterFeeder.setVoltage(volts);
    }
    public void reverseVariableVoltage(double volts) {
        m_SimpleShooterFeeder.setVoltage(volts);
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

    public Command SimpleShooterFeeder_stopCommand() {
        return Commands.runEnd(
            ()-> this.stop(),
            this::stop,
            this);
    }

    public Command zeroEncoderCommand() {
        return Commands.runOnce(() -> this.zeroEncoderValue(), this);
    }

    public Command simpleShooterFeeder_forwardsVaribleCommand(DoubleSupplier volts) {
        return Commands.runEnd(
            ()-> this.forwardsVariableVoltage(volts.getAsDouble()),
            this::stop,
            this);
    }

    public Command simpleShooterFeeder_reverseVariableCommand(DoubleSupplier volts) {
        return Commands.runEnd(
            ()-> this.reverseVariableVoltage(volts.getAsDouble()),
            this::stop,
            this);
    }






}
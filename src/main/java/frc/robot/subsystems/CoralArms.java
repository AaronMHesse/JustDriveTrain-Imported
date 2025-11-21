package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArms extends SubsystemBase {
    
    private TalonFX m_motor = new TalonFX(10);
    private TalonFXConfiguration m_config = new TalonFXConfiguration();
    private final MotionMagicVoltage m_position = new MotionMagicVoltage(0).withSlot(0);

public CoralArms() {

    //PID Setup
    m_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_config.Slot0.kP = 150;
    m_config.Slot0.kD = 0.1;
    m_config.Slot0.kA = 0.01;
    m_config.Slot0.kS = 1;
    m_config.Slot0.kV = 1;
    m_config.Slot0.kG = 0.2;

    m_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .20834; //~75 on Shuffleboard

    m_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    m_config.MotionMagic.MotionMagicAcceleration = 300;
    m_config.MotionMagic.MotionMagicCruiseVelocity = 1750;
    m_config.MotionMagic.MotionMagicJerk = 750;

    m_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    m_config.Feedback.FeedbackRemoteSensorID = 22;
    m_config.Feedback.SensorToMechanismRatio = 4;
    m_config.Feedback.RotorToSensorRatio = 80;

    m_motor.getConfigurator().apply(m_config);
    // m_encoder.setPosition(0);
}

    @Override
    public void periodic() {
        double ArmPosition = m_motor.getPosition().getValueAsDouble() * 360;
        SmartDashboard.putNumber("CoralPos.", Math.round(ArmPosition * 100) / 100);
        SmartDashboard.putNumber("Coral Kraken Temp", m_motor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Coral Kraken Current", Math.round(m_motor.getRotorVelocity().getValueAsDouble() * 10) / 10);
        SmartDashboard.putNumber("Coral Kraken ID", m_motor.getDeviceID());
        SmartDashboard.putNumber("Coral Kraken Velocity", Math.round(m_motor.getVelocity().getValueAsDouble() * 10) / 10);
    }

    //Positioning
    public void v_jog(double speed) {
        m_motor.set(speed);
    }

    public void v_setpoint(double deg) {
        m_motor.setControl(m_position.withPosition(deg/360));
    }
}
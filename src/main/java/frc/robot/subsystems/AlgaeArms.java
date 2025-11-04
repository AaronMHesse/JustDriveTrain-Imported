package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArms extends SubsystemBase {

    private TalonFX m_motor = new TalonFX(12);
    private TalonFXConfiguration m_config = new TalonFXConfiguration();
    private final MotionMagicVoltage m_position = new MotionMagicVoltage(0).withSlot(0);
    private DecimalFormat df = new DecimalFormat("#.##");

public AlgaeArms () {

    //PID Setup
    m_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_config.Slot0.kP = 200;
    m_config.Slot0.kG = 0.25;
    m_config.Slot0.kA = 0.01;
    m_config.Slot0.kD = 0.1;
    m_config.Slot0.kS = 1;
    m_config.Slot0.kV = 1;

    m_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.20834; //~76 on Shuffleboard
    m_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    m_config.MotionMagic.MotionMagicAcceleration = 500;
    m_config.MotionMagic.MotionMagicCruiseVelocity = 2500;
    m_config.MotionMagic.MotionMagicJerk = 2000;

    m_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    m_config.Feedback.FeedbackRemoteSensorID = 21;
    m_config.Feedback.SensorToMechanismRatio = 4;
    m_config.Feedback.RotorToSensorRatio = 80;

    m_motor.getConfigurator().apply(m_config);
}

    @Override
    public void periodic() {
        double ArmPosition = m_motor.getPosition().getValueAsDouble() * 360;
        SmartDashboard.putString("AlgaePos", df.format(ArmPosition));
    }

    //Positioning
    public void v_jog(double speed) {
        m_motor.set(speed);
    }

    public void v_setpoint(double deg) {
        m_motor.setControl(m_position.withPosition(deg/360));
    }
}
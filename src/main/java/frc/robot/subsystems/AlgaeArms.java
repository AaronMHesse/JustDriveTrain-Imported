package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Newton;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArms extends SubsystemBase {

    TalonFX m_kraken = new TalonFX(12);
    TalonFXConfiguration m_config = new TalonFXConfiguration();
    CANcoder m_encoder = new CANcoder(21);
    final MotionMagicDutyCycle m_position = new MotionMagicDutyCycle(0).withSlot(0);

// private final SparkFlex m_armsMotor = new SparkFlex(Constants.MyConstants.kAlgaeArm, MotorType.kBrushless);
// private SparkFlexConfig m_config = new SparkFlexConfig();

XboxController m_driverController = new XboxController(0);


public AlgaeArms () {

    // m_config.Slot0.kP = 0.1;
    // m_config.Slot0.kI = 0;
    // m_config.Slot0.kD = 0;
    // m_config.Voltage.withPeakForwardVoltage(Volts.of(16)).withPeakReverseVoltage(Volts.of(-16));

    m_config.MotionMagic.MotionMagicAcceleration = 40;
    m_config.MotionMagic.MotionMagicCruiseVelocity = 80;
    m_config.MotionMagic.MotionMagicJerk = 160;

    m_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    m_config.Feedback.SensorToMechanismRatio = 4;
    m_config.Feedback.RotorToSensorRatio = 80;

    m_kraken.getConfigurator().apply(m_config);

    //PID SETUP
    // m_config
    // .inverted(false)
    // .idleMode(IdleMode.kBrake);
    // m_config.externalEncoder
    // .positionConversionFactor(45)
    // .velocityConversionFactor(80);
    // m_config.closedLoop
    // .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
    // .pid(0.1, 0, 0.75)
    // .outputRange(-0.6, 0.6);

    // m_armsMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  
}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaePos", m_kraken.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("AlgaeAbsolutePos", m_encoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    public void v_manualPos(double speed) {
        m_kraken.set(speed);
    }

    public void v_setPos(double deg) {
        m_kraken.setControl(m_position.withPosition(deg / 360));
    }

    // public Command c_autoAlgaeArmsJog(double speed) {
    //     return new InstantCommand(() -> m_armsMotor.set(speed));
    // }

    // public void c_algaeArmsJog(double speed) {
    //     m_armsMotor.set(speed);
    // }


    //     //POSITIONING
    //   public Command c_algaeBarge() {
    //     return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(10, ControlType.kPosition), this);
    // }

    // public Command c_algaeArmsResting() {
    //     return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(30, ControlType.kPosition), this);
    // }

    // public Command c_algaeArmsProcessor() {
    //     return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(65, ControlType.kPosition), this);
    // }

    // public Command c_algaeArmsIntake() {
    //     return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(74, ControlType.kPosition), this);
    // }


    
    // public void v_algaeArmsResting() {
    //     m_armsMotor.getClosedLoopController().setReference(30, ControlType.kPosition);
    // }

    // public void v_algaeArmsProcessor() {
    //     m_armsMotor.getClosedLoopController().setReference(65, ControlType.kPosition);
    // }

    // public void v_algaeArmsIntake() {
    //     m_armsMotor.getClosedLoopController().setReference(74, ControlType.kPosition);
    // }


    // //RESET METHOD
    // public Command c_resetAlgaeEncoder() {
    //     return new InstantCommand(() -> {
    //         m_armsMotor.getExternalEncoder().setPosition(0);
    //     }, this);
    // }
}
package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArms extends SubsystemBase {

private final SparkFlex m_armsMotor = new SparkFlex(Constants.MyConstants.kAlgaeArm, MotorType.kBrushless);
private SparkFlexConfig m_config = new SparkFlexConfig();
// public SparkFlexExternalEncoder algae_encoder;
// private ExternalEncoderConfig m_configTest = new ExternalEncoderConfig();

XboxController m_driverController = new XboxController(0);


public AlgaeArms () {

    //PID SETUP
    m_config
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    m_config.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(80);
    m_config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.1, 0, 0.75)
    .outputRange(-0.2, 0.2);

    m_armsMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  
}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Arms Position", m_armsMotor.getEncoder().getPosition());
    }


    //ALGAE ARMS//
    public Command c_autoAlgaeArmsJog(double speed) {
        return new InstantCommand(() -> m_armsMotor.set(speed));
    }

    public void c_algaeArmsJog(double speed) {
        m_armsMotor.set(speed);
    }


        //POSITIONING
    public Command c_algaeArmsResting() {
        return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(0, ControlType.kPosition), this);
    }

    public Command c_autoAlgaeArmsHoldResting() {
        return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(30, ControlType.kPosition), this);
    }

    public Command c_algaeBarge() {
        return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(5, ControlType.kPosition), this);
    }

    public Command c_autoAlgaeArmsProcessor() {
        return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(45, ControlType.kPosition), this);
    }

    public Command c_autoAlgaeArmsIntake() {
        return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(63, ControlType.kPosition), this);
    }

    public void c_algaeArmsHoldResting() {
        m_armsMotor.getClosedLoopController().setReference(30, ControlType.kPosition);
    }

    public void c_algaeArmsProcessor() {
        m_armsMotor.getClosedLoopController().setReference(45, ControlType.kPosition);
    }

    public void c_algaeArmsIntake() {
        m_armsMotor.getClosedLoopController().setReference(63, ControlType.kPosition);
    }


    //RESET METHOD
    public Command c_resetAlgaeEncoder() {
        return new InstantCommand(() -> {
            m_armsMotor.getEncoder().setPosition(0);
        }, this);
    }
}
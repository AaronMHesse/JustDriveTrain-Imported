package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

private final SparkMax m_topWheels = new SparkMax(13, MotorType.kBrushless);
private final SparkMax m_bottomWheels = new SparkMax(14, MotorType.kBrushless);
private final SparkFlex m_armsMotor = new SparkFlex(Constants.MyConstants.kAlgaeArm, MotorType.kBrushless);
private SparkFlexConfig m_config = new SparkFlexConfig();

public AlgaeSubsystem () {

    //PID SETUP
    m_config
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    m_config.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(45);
    m_config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.1, 0, 0.75)
    .outputRange(-0.5, 0.5);

    m_armsMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

    @Override
    public void periodic() {

    }


    //ALGAE ARMS//
    public Command c_autoAlgaeArmsRun(double speed) {
        return new InstantCommand(() -> m_armsMotor.set(speed));
    }

    public void c_algaeArmsRun(double speed) {
        m_armsMotor.set(speed);
    }


        //POSITIONING
    public Command c_autoAlgaeArmsResting() {
        return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(0, ControlType.kPosition));
    }

    public Command c_autoAlgaeArmsHoldResting() {
        return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(30, ControlType.kPosition));
    }

    public Command c_autoAlgaeArmsProcessor() {
        return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(55, ControlType.kPosition));
    }

    public Command c_autoAlgaeArmsIntake() {
        return new InstantCommand(() -> m_armsMotor.getClosedLoopController().setReference(70, ControlType.kPosition));
    }

    public void c_algaeArmsResting() {
        m_armsMotor.getClosedLoopController().setReference(0, ControlType.kPosition);
        
    }

    public void c_algaeArmsHoldResting() {
        m_armsMotor.getClosedLoopController().setReference(30, ControlType.kPosition);
    }

    public void c_algaeArmsProcessor() {
        m_armsMotor.getClosedLoopController().setReference(55, ControlType.kPosition);
    }

    public void c_algaeArmsIntake() {
        m_armsMotor.getClosedLoopController().setReference(70, ControlType.kPosition);
    }
 


    // ALGAE WHEELS//
    public Command c_autoAlgaeWheelsRun(double speed) {

        return new InstantCommand(() -> m_topWheels.set(speed), this);
    }

    public void c_algaeWheelsRun(double speed) {
        m_topWheels.set(speed);
        m_bottomWheels.set(-speed);
    }



    //RESET METHODS
    public void c_freeArm() {
        m_config
        .idleMode(IdleMode.kCoast)
        .inverted(false);

        m_armsMotor.configure(m_config, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
    }

    public void c_resetEncoder() {
        m_armsMotor.getEncoder().setPosition(0);

        m_config
        .idleMode(IdleMode.kBrake)
        .inverted(false);

        m_armsMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
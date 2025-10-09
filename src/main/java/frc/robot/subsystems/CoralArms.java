package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArms extends SubsystemBase {
    
private final SparkFlex m_clawArm = new SparkFlex(Constants.MyConstants.kCoralClawArm, MotorType.kBrushless);
private SparkFlexConfig m_clawConfig = new SparkFlexConfig();

XboxController m_driverController = new XboxController(0);


public CoralArms() {

    //PID SETUP
    m_clawConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    m_clawConfig.externalEncoder
    .positionConversionFactor(45)
    .velocityConversionFactor(80);
    m_clawConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
    .pid(0.15, 0, 0.55)
    .outputRange(-0.6, 0.6);

    m_clawArm.configure(m_clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral Arm Position", m_clawArm.getExternalEncoder().getPosition());
    }

    
    public void v_coralArmJog(double speed) {
        m_clawArm.set(-speed);
    }


        //POSITIONING
    public Command c_coralArmResting() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(-9, ControlType.kPosition), this);
    }

    public Command c_autoCoralArmHoldResting() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(-27, ControlType.kPosition), this);
    }

    public Command c_coralTrough() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(-41.5, ControlType.kPosition), this);
    }

    public Command c_autoCoralArmIntake() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(-78, ControlType.kPosition), this);
    }

    public Command c_coralReef() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(-15, ControlType.kPosition), this);
    }

    public Command c_coralArmStation() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(-6.5, ControlType.kPosition), this);
    }

    public void v_coralArmHoldResting() {
        m_clawArm.getClosedLoopController().setReference(-9, ControlType.kPosition);
    }

    public void v_coralArmIntake() {
        m_clawArm.getClosedLoopController().setReference(-78, ControlType.kPosition);
    }


   
    //RESET METHOD
    public Command c_resetCoralEncoder() {
        return new InstantCommand(() -> {
        m_clawArm.getExternalEncoder().setPosition(0);
        }, this);
    }
}
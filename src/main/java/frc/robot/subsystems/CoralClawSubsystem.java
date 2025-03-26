package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.MyConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralClawSubsystem extends SubsystemBase {
    
private final SparkFlex m_clawArm = new SparkFlex(Constants.MyConstants.kCoralClawArm, MotorType.kBrushless);
private final SparkMax m_clawWheels = new SparkMax(Constants.MyConstants.kCoralClawWheels, MotorType.kBrushless);
private SparkMaxConfig m_clawWheelsConfig = new SparkMaxConfig();
private SparkFlexConfig m_clawConfig = new SparkFlexConfig();

XboxController m_driverController = new XboxController(0);


public CoralClawSubsystem() {

    m_clawWheelsConfig.inverted(true);
    m_clawWheels.configure(m_clawWheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //PID SETUP
    m_clawConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    m_clawConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(80);
    m_clawConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(0.15, 0, 0.55, 0.01)
    .outputRange(-0.35, 0.35);

    m_clawArm.configure(m_clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral Arm Position", m_clawArm.getEncoder().getPosition());
    }

    
    //CORAL ARM

    public void c_coralArmJog(double speed) {
        m_clawArm.set(-speed);
    }


        //POSITIONING
    public Command c_coralArmResting() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(0, ControlType.kPosition), this);
    }

    public Command c_autoCoralArmHoldResting() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(8.5, ControlType.kPosition), this);
    }

    public Command c_autoCoralArmIntake() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(73, ControlType.kPosition), this);
    }

    public Command c_coralL2() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(30, ControlType.kPosition), this);
    }

    public Command c_coralArmStation() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(7.8, ControlType.kPosition), this);
    }

    public Command c_coralL4() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(48, ControlType.kPosition), this);
    }

    public void c_coralArmHoldResting() {
        m_clawArm.getClosedLoopController().setReference(8.5, ControlType.kPosition);
    }

    public void c_coralArmIntake() {
        m_clawArm.getClosedLoopController().setReference(68, ControlType.kPosition);
    }

    
  
    //CORAL WHEELS
    public Command c_coralWheelsRun(double speed) {
        return new InstantCommand(() -> m_clawWheels.set(speed), this);
    }

    // public void c_coralWheelsOutput() {
    //         if(MyConstants.kTriggerR >= 0.5) {
    //         m_clawWheels.set(0.4);
    //         } else if (m_driverController.getRightBumperButtonPressed()){
    //             m_clawWheels.set(-0.6);
    //         } else {
    //             m_clawWheels.set(0);
    //         }
    // }


   
    //RESET METHOD
    public Command c_resetCoralEncoder() {
        return new InstantCommand(() -> {
        m_clawArm.getEncoder().setPosition(0);
        });
    }
}
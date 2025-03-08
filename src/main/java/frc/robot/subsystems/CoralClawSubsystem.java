package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
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

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralClawSubsystem extends SubsystemBase {
    
private final SparkFlex m_clawArm = new SparkFlex(Constants.MyConstants.kCoralClawArm, MotorType.kBrushless);
private final SparkMax m_clawWheels = new SparkMax(Constants.MyConstants.kCoralClawWheels, MotorType.kBrushless);
private SparkFlexConfig m_clawConfig = new SparkFlexConfig();
private RelativeEncoder m_armEncoder;

public CoralClawSubsystem() {

    // //PID SETUP
    m_clawConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    m_clawConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(5);
    m_clawConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(0.05, 0, 0.55, 0.5)
    .outputRange(-0.1, 0.1);

    m_clawArm.configure(m_clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

    @Override
    public void periodic() {

    }

    
    //CORAL ARM//
    public Command c_autoCoralArmRun(double speed) {
        
        return new InstantCommand(() -> m_clawArm.set(speed), this);
    }

    public Command c_autoCoralArmSetResting() {
        return new InstantCommand(() -> m_clawArm.getClosedLoopController().setReference(0, ControlType.kPosition));
    }

    public void c_coralArmRun(double speed) {
        m_clawArm.set(speed);
    }

    public void c_coralArmSetResting() {
        m_clawArm.getClosedLoopController().setReference(0, ControlType.kPosition);
    }


    //CORAL WHEELS//
    public Command c_autoCoralWheelsRun(double speed) {
        return new InstantCommand(() -> m_clawWheels.set(speed), this);
    }

    public void c_coralWheelsRun(double speed) {
        m_clawWheels.set(speed);
    }

    public void c_coralArmSetIntake() {
        m_clawArm.getClosedLoopController().setReference(45, ControlType.kPosition);
    }
}

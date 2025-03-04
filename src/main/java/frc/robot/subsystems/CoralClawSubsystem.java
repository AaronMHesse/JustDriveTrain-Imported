package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
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
private SparkFlexConfig m_config = new SparkFlexConfig();
private RelativeEncoder m_armEncoder;

public CoralClawSubsystem() {

    //PID SETUP
    m_config
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    m_config.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(5);
    m_config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(0.05, 0, 0.55, -.5)
    .outputRange(-0.5, 0.5);

    m_clawArm.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

    @Override
    public void periodic() {

    }

    
    //CORAL ARM//
    public Command c_autoCoralClawArmRun(double speed) {
        
        return new InstantCommand(() -> m_clawArm.set(speed), this);
    }

    public void c_coralClawArmRun(double speed) {
        m_clawArm.set(speed);
    }


    //CORAL WHEELS//
    public Command c_autoCoralClawWheelsRun(double speed) {
        
        return new InstantCommand(() -> m_clawWheels.set(speed), this);
    }

    public void c_coralClawWheelsRun(double speed) {
        m_clawWheels.set(speed);
    }
}

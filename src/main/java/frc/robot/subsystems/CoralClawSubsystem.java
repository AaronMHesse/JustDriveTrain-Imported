package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
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
    
    private final IdleMode m_coralMotorIdle = IdleMode.kBrake;
    private final SparkFlexConfig m_coralArmMotor;
    private final RelativeEncoder m_coralArmEncoder;
    private final SparkFlex m_coralClawArm;
    private final SparkMax m_coralClawWheels;
    private final SparkClosedLoopController m_coralArmClosedLoopController;

    public CoralClawSubsystem() {
        m_coralClawArm = new SparkFlex(Constants.MyConstants.kCoralClawArm, MotorType.kBrushless);
        m_coralClawWheels = new SparkMax(Constants.MyConstants.kCoralClawWheels, MotorType.kBrushless);

        m_coralArmMotor = new SparkFlexConfig();
        m_coralArmEncoder = m_coralClawArm.getEncoder();
        m_coralArmClosedLoopController = m_coralClawArm.getClosedLoopController();

        //CORAL ARM FEEDBACK
        m_coralArmMotor.encoder
        .velocityConversionFactor(1)
        .positionConversionFactor(1);

        //PID
        m_coralArmMotor.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

            .p(0.5)
            .i(0)
            .d(0)
            .outputRange(-1, 1)

            .p(0.01, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5676, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void periodic() {

    }

    
    //CORAL ARM//
    public Command c_autoCoralClawArmRun(double speed) {
        
        return new InstantCommand(() -> m_coralClawArm.set(speed), this);
    }

    public void c_coralClawArmRun(double speed) {
        m_coralClawArm.set(speed);
    }


    //CORAL WHEELS//
    public Command c_autoCoralClawWheelsRun(double speed) {
        
        return new InstantCommand(() -> m_coralClawWheels.set(speed), this);
    }

    public void c_coralClawWheelsRun(double speed) {
        m_coralClawWheels.set(speed);
    }
}

package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
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
    
private final SparkMax m_topAlgaeWheels;
private final SparkMax m_followerTopAlgaeWheels;
private final SparkFlex m_algaeArms;
private final SparkFlexConfig m_algaeArmsMotor;
private final AbsoluteEncoder m_algaeArmsEncoder;
private final SparkClosedLoopController m_algaeArmsClosedLoopController;

public AlgaeSubsystem () {
m_topAlgaeWheels = new SparkMax(13, MotorType.kBrushless);
m_followerTopAlgaeWheels = new SparkMax(14, MotorType.kBrushless);
m_algaeArms = new SparkFlex(Constants.MyConstants.kAlgaeArm, MotorType.kBrushless);

m_algaeArms.setInverted(true);

//m_algaeArmsMotor = new SparkFlexConfig();
m_algaeArmsMotor = new SparkFlexConfig();
m_algaeArmsEncoder = m_algaeArms.getAbsoluteEncoder();
m_algaeArmsClosedLoopController = m_algaeArms.getClosedLoopController();


//ALGAE ARMS MOTOR FEEDBACK
m_algaeArmsMotor.encoder
.velocityConversionFactor(1)
.positionConversionFactor(1);


//PID
m_algaeArmsMotor.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)

//     // .p(0.1)
//     // .i(0)
//     // .d(0.75)
//     // .outputRange(-0.5, 0.5)

    .p(0.1)
    .i(0)
    .d(0.75)
    .velocityFF(0)
    .outputRange(-0.1, 0.1);
}


    @Override
    public void periodic() {

    }


    //ALGAE ARMS//
    public Command c_autoAlgaeArmRun(double speed) {

        return new InstantCommand(() -> m_algaeArms.set(speed), this);
    }

    public void c_algaeArmRun(double speed) {
        m_algaeArms.set(speed);
    }


    // ALGAE WHEELS//
    public Command c_autoAlgaeWheelsRun(double speed) {

        return new InstantCommand(() -> m_topAlgaeWheels.set(speed), this);
    }

    public void c_algaeWheelsRun(double speed) {
        m_topAlgaeWheels.set(speed);
        m_followerTopAlgaeWheels.set(-speed);
    }

    public void c_algaeArmStow() {
        m_algaeArmsClosedLoopController.setReference(0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        
    }

    public void c_algaeArmUp() {
        m_algaeArmsClosedLoopController.setReference(-40, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }
}
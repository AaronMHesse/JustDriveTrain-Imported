package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
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
private final RelativeEncoder m_algaeArmsEncoder;
private final SparkClosedLoopController m_algaeArmsClosedLoopController;

public AlgaeSubsystem () {
m_topAlgaeWheels = new SparkMax(13, MotorType.kBrushless);
m_followerTopAlgaeWheels = new SparkMax(14, MotorType.kBrushless);
m_algaeArms = new SparkFlex(Constants.MyConstants.kAlgaeArm, MotorType.kBrushless);

m_algaeArms.setInverted(true);

m_algaeArmsMotor = new SparkFlexConfig();
m_algaeArmsEncoder = m_algaeArms.getEncoder();
m_algaeArmsClosedLoopController = m_algaeArms.getClosedLoopController();


//ALGAE ARMS MOTOR FEEDBACK
m_algaeArmsMotor.encoder
.velocityConversionFactor(1)
.positionConversionFactor(1);


//PID
m_algaeArmsMotor.closedLoop
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


    //ALGAE ARM//
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

    
}

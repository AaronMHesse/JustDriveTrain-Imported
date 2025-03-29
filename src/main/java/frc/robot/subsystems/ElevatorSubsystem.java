package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

public final SparkFlex m_elevatorMotor1 = new SparkFlex(15, MotorType.kBrushless);
private final SparkFlex m_elevatorMotor2 = new SparkFlex(16, MotorType.kBrushless);
private SparkFlexConfig m_elevatorConfig = new SparkFlexConfig();
public boolean elevatorTooTall;

  public ElevatorSubsystem() {

    //PID SETUP
    m_elevatorConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    m_elevatorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(45);
    m_elevatorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.2, 0, 0.25)
    .outputRange(-0.5, 0.5);

    m_elevatorMotor2.configure(m_elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorMotor1.configure(m_elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", m_elevatorMotor1.getEncoder().getPosition());
  }

    //ELEVATOR POSITIONING
  public Command c_autoElevatorDown() {
    return new InstantCommand(() -> {
      m_elevatorMotor1.getClosedLoopController().setReference(0, ControlType.kPosition);
      m_elevatorMotor2.getClosedLoopController().setReference(0, ControlType.kPosition);
      elevatorTooTall = false;
    }, this);
  }

  public Command c_elevatorL3() {
    return new InstantCommand(() -> {
    m_elevatorMotor1.getClosedLoopController().setReference(84, ControlType.kPosition);
    m_elevatorMotor2.getClosedLoopController().setReference(-84, ControlType.kPosition);
    elevatorTooTall = true;
    }, this);
  }

  public Command c_elevatorL4() {
    return new InstantCommand(() -> {
      m_elevatorMotor1.getClosedLoopController().setReference(191, ControlType.kPosition);
      m_elevatorMotor2.getClosedLoopController().setReference(-191, ControlType.kPosition);
      elevatorTooTall = true;
    }, this);
  }

  public void c_elevatorDown() {
    m_elevatorMotor1.getClosedLoopController().setReference(0, ControlType.kPosition);
    m_elevatorMotor2.getClosedLoopController().setReference(0, ControlType.kPosition);
    elevatorTooTall = false;
  }

  //3 IN. UP
  public Command c_elevatorCoralStation() {
    return new InstantCommand(() -> {
      m_elevatorMotor1.getClosedLoopController().setReference(37, ControlType.kPosition);
      m_elevatorMotor2.getClosedLoopController().setReference(-37, ControlType.kPosition);
    }, this);
  }

  public Command c_elevatorL2() {
    return new InstantCommand(() -> {
      m_elevatorMotor1.getClosedLoopController().setReference(42.3, ControlType.kPosition);
      m_elevatorMotor2.getClosedLoopController().setReference(-42.3, ControlType.kPosition);
    }, this);
  }

  public void c_elevatorJog(double speed) {
    m_elevatorMotor1.set(speed);
    m_elevatorMotor2.set(-speed);
  }


  //RESET METHOD
  public Command c_resetElevatorEncoders() {
    return new InstantCommand(() -> {
      m_elevatorMotor1.getEncoder().setPosition(0);
      m_elevatorMotor2.getEncoder().setPosition(0);
    });
  }
}
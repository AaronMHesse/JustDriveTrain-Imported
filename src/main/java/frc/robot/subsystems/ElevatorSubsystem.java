package frc.robot.subsystems;

import java.text.DecimalFormat;
import java.util.Map;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private final SparkFlex m_elevatorMotor1 = new SparkFlex(15, MotorType.kBrushless);
  private final SparkFlex m_elevatorMotor2 = new SparkFlex(16, MotorType.kBrushless);
  private SparkFlexConfig m_motor1config = new SparkFlexConfig();
  private SparkFlexConfig m_motor2config = new SparkFlexConfig();
  private DecimalFormat df = new DecimalFormat("#.##");

  public ElevatorSubsystem() {

    //PID Setup
    m_motor1config
    .inverted(true)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(80)
    .closedLoopRampRate(0.45);
    m_motor1config.softLimit
    .forwardSoftLimitEnabled(true)
    .forwardSoftLimit(57)
    .reverseSoftLimitEnabled(true)
    .reverseSoftLimit(0);
    m_motor1config.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    m_motor1config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.09, 0, 0.25)
    .outputRange(-0.4, 0.6);

    m_motor2config
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(80)
    .closedLoopRampRate(0.45);
    m_motor2config.softLimit
    .forwardSoftLimitEnabled(true)
    .forwardSoftLimit(57)
    .reverseSoftLimitEnabled(true)
    .reverseSoftLimit(0);
    m_motor2config.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    m_motor2config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.09, 0, 0.25)
    .outputRange(-0.4, 0.6);

    m_elevatorMotor1.configure(m_motor1config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorMotor2.configure(m_motor2config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Shuffleboard.getTab("SmartDashboard")
    .add("Test", m_elevatorMotor1.getEncoder().getPosition())
    .withWidget(BuiltInWidgets.kNumberBar)
    .withSize(1, 2)
    .withProperties(Map.of("Min", 0, "Max", 57))
    .withPosition(0, 0);
  }

  @Override
  public void periodic() {
    double elevatorPosition = m_elevatorMotor1.getEncoder().getPosition();
    SmartDashboard.putString("Elevator Position", df.format(elevatorPosition));
    SmartDashboard.updateValues();
  }

    //Positioning
  public void v_setpoint(double pos) {
    m_elevatorMotor1.getClosedLoopController().setReference(pos, ControlType.kPosition);
    m_elevatorMotor2.getClosedLoopController().setReference(pos, ControlType.kPosition);
  }

  public void v_jog(double speed) {
    m_elevatorMotor1.set(speed);
    m_elevatorMotor2.set(speed);
  }

  //Reset Encoders
  public Command c_resetElevatorEncoders() {
    return new InstantCommand(() -> {
      m_elevatorMotor1.getEncoder().setPosition(0);
      m_elevatorMotor2.getEncoder().setPosition(0);
    });
  }
}
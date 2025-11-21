package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
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

public class CoralRotator extends SubsystemBase {

  private final SparkFlex m_wrist = new SparkFlex(18, MotorType.kBrushless);
  private CANcoder m_CANCoder = new CANcoder(20);
  private SparkFlexConfig m_rotatorConfig = new SparkFlexConfig();
  public static boolean IsClaw90Deg;

  public CoralRotator() {

    //PID Setup
    m_rotatorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    m_rotatorConfig.softLimit
    .forwardSoftLimitEnabled(true)
    .forwardSoftLimit(90)
    .reverseSoftLimitEnabled(true)
    .reverseSoftLimit(0);
    m_rotatorConfig.encoder
    .positionConversionFactor(15)
    .velocityConversionFactor(1);
    m_rotatorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.15, 0, 0)
    .outputRange(-0.25, 0.25);

    m_wrist.configure(m_rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_wrist.getEncoder().setPosition(m_CANCoder.getAbsolutePosition().getValueAsDouble() * 375);
  }

  //Positioning
  public void v_clawHome() {
    m_wrist.getClosedLoopController().setReference(0, ControlType.kPosition);
    IsClaw90Deg = false;
  }

  public void v_claw90Deg() {
    m_wrist.getClosedLoopController().setReference(100, ControlType.kPosition);
    IsClaw90Deg = true;
  }

  public Command c_clawHome() {
    return new InstantCommand(() -> {
      m_wrist.getClosedLoopController().setReference(0, ControlType.kPosition);
      IsClaw90Deg = false;
    }, this);
  }

  public Command c_claw90Deg() {
    return new InstantCommand(() -> {
      m_wrist.getClosedLoopController().setReference(90, ControlType.kPosition);
      IsClaw90Deg = true;
    }, this);
  }

  @Override
  public void periodic() {
    double WristPosition = m_wrist.getEncoder().getPosition();
    SmartDashboard.putNumber("Wrist Pos", Math.round(WristPosition * 100) / 100);
    SmartDashboard.putNumber("Wrist NeoV Temp", m_wrist.getMotorTemperature());
    SmartDashboard.putNumber("Wrist NeoV Current", Math.round(m_wrist.getOutputCurrent() * 10) / 10);
    SmartDashboard.putNumber("Wrist NeoV ID", m_wrist.getDeviceId());
  }
}

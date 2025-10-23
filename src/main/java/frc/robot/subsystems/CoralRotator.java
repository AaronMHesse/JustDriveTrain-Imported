package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
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

  private final SparkFlex m_clawRotator = new SparkFlex(18, MotorType.kBrushless);
  private CANcoder m_CANCoder = new CANcoder(25);
  private RelativeEncoder m_relativeEncoder = m_clawRotator.getEncoder();
  double absolutePosition;
  private SparkFlexConfig m_rotatorConfig = new SparkFlexConfig();
  public static boolean IsClaw90Deg;

  public CoralRotator() {

    m_rotatorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    m_rotatorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    m_rotatorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.15, 0, 0)
    .outputRange(-0.25, 0.25);

    m_clawRotator.configure(m_rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_clawRotator.getEncoder().setPosition(m_CANCoder.getAbsolutePosition().getValueAsDouble() * 25);
  }

  public void v_clawHome() {
    m_clawRotator.getClosedLoopController().setReference(0, ControlType.kPosition);
    IsClaw90Deg = false;
  }

  public void v_claw90Deg() {
    m_clawRotator.getClosedLoopController().setReference(6, ControlType.kPosition);
    IsClaw90Deg = true;
  }

  public Command c_clawHome() {
    return new InstantCommand(() -> {
      m_clawRotator.getClosedLoopController().setReference(0, ControlType.kPosition);
      IsClaw90Deg = false;
    }, this);
  }

  public Command c_claw90Deg() {
    return new InstantCommand(() -> {
      m_clawRotator.getClosedLoopController().setReference(6, ControlType.kPosition);
      IsClaw90Deg = true;
    }, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Relative", m_clawRotator.getEncoder().getPosition());
    SmartDashboard.putNumber("Wrist Absolute", m_CANCoder.getAbsolutePosition().getValueAsDouble());
  }
}

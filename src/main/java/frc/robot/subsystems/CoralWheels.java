package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralWheels extends SubsystemBase {

private final SparkMax m_bottomWheels = new SparkMax(17, MotorType.kBrushless);
private final SparkMax m_topWheels = new SparkMax(11, MotorType.kBrushless);
private final SparkMax m_insideWheel = new SparkMax(19, MotorType.kBrushless);
private final SparkMaxConfig m_config = new SparkMaxConfig();

  public CoralWheels() {
    m_config
    .smartCurrentLimit(40)
    .idleMode(IdleMode.kBrake)
    .inverted(false);

    m_topWheels.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_insideWheel.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_bottomWheels.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void v_0degSpin(double top, double bot, double mid, double groundTranslation) {
    if (CoralRotator.IsClaw90Deg == false) {
    m_topWheels.set(top);
    m_bottomWheels.set(bot);
    m_insideWheel.set(-mid);
    } else {
      m_topWheels.set(top - groundTranslation);
      m_bottomWheels.set(-bot + groundTranslation);
    }
  }

  public void v_90degSpin(double speed) {
      m_topWheels.set(-speed);
      m_bottomWheels.set(speed);
  }

  public Command c_stopAllWheels() {
    return new InstantCommand(() -> {
      m_topWheels.stopMotor();
      m_bottomWheels.stopMotor();
      m_insideWheel.stopMotor();
    }, this);
  }

  public Command c_outsideCoralWheelsRun(double speed) {
    return new InstantCommand(() -> {
      m_topWheels.set(speed);
      m_bottomWheels.set(-speed);
    }, this);
  }

  public Command c_coral90DegOutput() {
    return new InstantCommand(() -> {
      m_topWheels.set(0.7);
      m_bottomWheels.set(0.7);
      m_insideWheel.set(-1);
    }, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Top Coral Neo Temp", m_topWheels.getMotorTemperature());
    SmartDashboard.putNumber("Top Coral Neo Current", Math.round(m_topWheels.getOutputCurrent() * 10) / 10);
    SmartDashboard.putNumber("Top Coral Neo ID", m_topWheels.getDeviceId());
    SmartDashboard.putNumber("Top Coral Neo Velocity", Math.round(m_topWheels.getEncoder().getVelocity() * 10) / 10);

    SmartDashboard.putNumber("Middle Coral Neo Temp", m_insideWheel.getMotorTemperature());
    SmartDashboard.putNumber("Middle Coral Neo Current", Math.round(m_insideWheel.getOutputCurrent() * 10) / 10);
    SmartDashboard.putNumber("Middle Coral Neo ID", m_insideWheel.getDeviceId());
    SmartDashboard.putNumber("Middle Coral Neo Velocity", Math.round(m_insideWheel.getEncoder().getVelocity() * 10) / 10);

    SmartDashboard.putNumber("Bottom Coral Neo Temp", m_bottomWheels.getMotorTemperature());
    SmartDashboard.putNumber("Bottom Coral Neo Current", Math.round(m_bottomWheels.getOutputCurrent() * 10) / 10);
    SmartDashboard.putNumber("Bottom Coral Neo ID", m_bottomWheels.getDeviceId());
    SmartDashboard.putNumber("Bottom Coral Neo Velocity", Math.round(m_bottomWheels.getEncoder().getVelocity() * 10) / 10);
  }

}

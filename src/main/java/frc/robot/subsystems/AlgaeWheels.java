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

public class AlgaeWheels extends SubsystemBase {

private final SparkMax m_topWheels = new SparkMax(13, MotorType.kBrushless);
private final SparkMax m_bottomWheels = new SparkMax(14, MotorType.kBrushless);
private final SparkMaxConfig m_config = new SparkMaxConfig();

  public AlgaeWheels() {
    m_config
    .smartCurrentLimit(40)
    .idleMode(IdleMode.kBrake)
    .inverted(false);

    m_topWheels.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_topWheels.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
    public Command c_autoAlgaeWheelsRun(double speed) {
        return new InstantCommand(() -> {
          m_topWheels.set(speed);
          m_bottomWheels.set(-speed);
        }, this);
    }

    public void v_algaeWheelsRun(double speed) {
        m_topWheels.set(speed);
        m_bottomWheels.set(-speed);
    }

    public void v_algaeWheelsOutput(double top, double bottom) {
        m_topWheels.set(top);
        m_bottomWheels.set(-bottom);
    }
    
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Top Algae Neo Temp", m_topWheels.getMotorTemperature());
    SmartDashboard.putNumber("Top Algae Neo Current", Math.round(m_topWheels.getOutputCurrent() * 10) / 10);
    SmartDashboard.putNumber("Top Algae Neo ID", m_topWheels.getDeviceId());
    SmartDashboard.putNumber("Top Algae Neo Velocity", Math.round(m_topWheels.getEncoder().getVelocity() * 10) / 10);

    SmartDashboard.putNumber("Bottom Algae Neo Temp", m_bottomWheels.getMotorTemperature());
    SmartDashboard.putNumber("Bottom Algae Neo Current", Math.round(m_bottomWheels.getOutputCurrent() * 10) / 10);
    SmartDashboard.putNumber("Bottom Algae Neo ID", m_bottomWheels.getDeviceId());
    SmartDashboard.putNumber("Bottom Algae Neo Velocity", Math.round(m_bottomWheels.getEncoder().getVelocity() * 10) / 10);
  }
}

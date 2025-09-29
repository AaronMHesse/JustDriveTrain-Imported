package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralWheels extends SubsystemBase {

private final SparkMax m_topWheels = new SparkMax(11, MotorType.kBrushless);
private final SparkMax m_bottomWheels = new SparkMax(17, MotorType.kBrushless);
private final SparkMax m_insideWheel = new SparkMax(19, MotorType.kBrushless);
private SparkMaxConfig m_wheelConfig = new SparkMaxConfig();

  public CoralWheels() {
    m_wheelConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);

    m_topWheels.configure(m_wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_bottomWheels.configure(m_wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_insideWheel.configure(m_wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  //COMMANDS
  public void v_coralIntake() {
    m_topWheels.set(0.75);
    m_bottomWheels.set(-0.75);
  }

  public void v_coralOutput(double axis) {
    if (axis >= 0.5) {
    if (CoralRotator.IsClaw90Deg == false) {
      m_topWheels.set(0.5);
      m_bottomWheels.set(0.5);
      m_insideWheel.set(-0.7);
    } else {
      m_topWheels.set(-0.5);
      m_bottomWheels.set(0.5);
    }
  } else {
    m_topWheels.set(0);
    m_bottomWheels.set(0);
    m_insideWheel.set(0);
  }
  }

  public Command c_outsideCoralWheelsRun(double speed) {
    return new InstantCommand(() -> {
      m_topWheels.set(speed);
      m_bottomWheels.set(-speed);
    }, this);
  }

  public Command c_coral90DegOutput() {
    return new InstantCommand(() -> {
      m_topWheels.set(0.5);
      m_bottomWheels.set(0.5);
      m_insideWheel.set(-0.7);
    }, this);
  }

  @Override
  public void periodic() {

  }
}

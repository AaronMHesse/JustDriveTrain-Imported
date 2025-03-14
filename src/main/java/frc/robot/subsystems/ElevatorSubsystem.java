// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

private final SparkFlex m_elevatorMotor1 = new SparkFlex(15, MotorType.kBrushless);
private final SparkFlex m_elevatorMotor2 = new SparkFlex(16, MotorType.kBrushless);
private SparkFlexConfig m_elevatorConfig = new SparkFlexConfig();

  public ElevatorSubsystem() {

m_elevatorMotor1.setInverted(true);
m_elevatorMotor2.setInverted(true);

    //PID SETUP
    m_elevatorConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    m_elevatorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(45);
    m_elevatorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.1, 0, 0.75)
    .outputRange(-0.2, 0.2);

    m_elevatorMotor2.configure(m_elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorMotor1.configure(m_elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

  }

//POSITIONING
  public void c_elevatorDown() {
    m_elevatorMotor1.getClosedLoopController().setReference(0, ControlType.kPosition);
    m_elevatorMotor2.getClosedLoopController().setReference(0, ControlType.kPosition);
  }

  public void c_elevatorUp() {
    m_elevatorMotor1.getClosedLoopController().setReference(40, ControlType.kPosition);
    m_elevatorMotor2.getClosedLoopController().setReference(-40, ControlType.kPosition);
  }

  public void c_elevatorJog(double speed) {
    m_elevatorMotor1.set(speed);
    m_elevatorMotor2.set(-speed);
  }

}

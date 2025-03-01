// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TempAlgae extends SubsystemBase {
  /** Creates a new TempAlgae. */

  private final SparkFlex m_motor = new SparkFlex(12, MotorType.kBrushless);
  private AbsoluteEncoder m_AbsoluteEncoder;
  private final double OFFSET = 0.0;
  private final double kP = 0.1;
  private final double kI = 0.0;
  private final double kD = 0.75;

  //private SparkClosedLoopController m_PIDController = m_motor.getClosedLoopController();
  private SparkFlexConfig m_config = new SparkFlexConfig();


  public TempAlgae() {
  
    m_config
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    m_config.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    m_config.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(kP, kI, kD);
    
    m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void c_algaeStowAway() {
    m_motor.getClosedLoopController().setReference(-Math.PI/2, ControlType.kPosition);
  }

  public void c_algaeIntake() {
    m_motor.getClosedLoopController().setReference(0, ControlType.kPosition);
  }

  public void c_algaeWheelsRun(double i) {
    System.out.println("Ow");
  }

  public void c_stop() {
    m_motor.getClosedLoopController().setReference(0, ControlType.kCurrent);
  }
}

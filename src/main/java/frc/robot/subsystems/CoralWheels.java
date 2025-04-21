// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralWheels extends SubsystemBase {

private final SparkMax m_clawWheels = new SparkMax(Constants.MyConstants.kCoralClawWheels, MotorType.kBrushless);
private SparkMaxConfig m_clawWheelsConfig = new SparkMaxConfig();

  public CoralWheels() {
    m_clawWheelsConfig.inverted(true);
    m_clawWheels.configure(m_clawWheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

    public Command c_autoCoralWheelsRun(double speed) {
        return new InstantCommand(() -> m_clawWheels.set(speed), this);
    }

    public void c_coralWheelsRun(double speed) {
        m_clawWheels.set(speed);
    }

    public void c_coralWheelsOutput(double axis, double speed) {
            if (axis >= 0.5) {
                m_clawWheels.set(speed);
                } else {
                    m_clawWheels.set(speed * 0);
                }

    }

  @Override
  public void periodic() {

  }
}

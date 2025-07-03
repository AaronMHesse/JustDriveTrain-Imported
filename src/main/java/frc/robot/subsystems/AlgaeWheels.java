package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeWheels extends SubsystemBase {

private final SparkMax m_topWheels = new SparkMax(13, MotorType.kBrushless);
private final SparkMax m_bottomWheels = new SparkMax(14, MotorType.kBrushless);

XboxController m_driverController = new XboxController(0);

  public AlgaeWheels() {

  }
  
    public Command c_autoAlgaeWheelsRun(double speed) {
        return new InstantCommand(() -> {
            m_topWheels.set(speed);
            m_bottomWheels.set(-speed);
        }, this);
    }

    public void c_algaeWheelsRun(double speed) {
        m_topWheels.set(speed);
        m_bottomWheels.set(-speed);
    }

    public void c_algaeWheelsOutput(double axis, double speed) {
            if (axis >= 0.5) {
                m_topWheels.set(speed - 0.25);
                m_bottomWheels.set(-speed - 0.15);
                } else {
                    m_topWheels.set(speed * 0);
                    m_bottomWheels.set(speed * 0);
                }
    }
    
  @Override
  public void periodic() {

  }
}

package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
    
    private final SparkMax m_coralWheels;

    public CoralSubsystem() {
        m_coralWheels = new SparkMax(Constants.kCoralWheels, MotorType.kBrushless);
    }

    @Override
    public void periodic() {

    }

    public Command c_autoCoralWheelRun(double speed) {
        
        return new InstantCommand(() -> m_coralWheels.set(speed), this);
    }

    public void c_coralWheelRun(double speed) {
        m_coralWheels.set(speed);
    }

    public void c_startCoralAxis(double axis, double speed) {
        if (axis >= 0.5) {
            m_coralWheels.set(speed);
        } else {
            m_coralWheels.set(0);
        }
    }

}

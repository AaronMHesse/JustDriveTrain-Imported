package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralSubsystem;

public class AutoCoralOutput extends ParallelCommandGroup {
    
public AutoCoralOutput(CoralSubsystem m_CoralSubsystem) {

    addCommands(
        m_CoralSubsystem.c_autoCoralWheelRun(0.5)
    );

    }

}

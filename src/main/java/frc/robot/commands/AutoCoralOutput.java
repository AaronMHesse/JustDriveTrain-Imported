package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralClawSubsystem;

public class AutoCoralOutput extends ParallelCommandGroup {
    
public AutoCoralOutput(CoralClawSubsystem m_coralClawSubsystem) {

    addCommands(
        m_coralClawSubsystem.c_autoCoralWheelsRun(0.5)
    );

    }

}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralClawSubsystem;

public class FailSafe extends ParallelCommandGroup {

  public FailSafe(AlgaeSubsystem m_AlgaeSubsystem, CoralClawSubsystem m_coralClawSubsystem) {

  addCommands(
    m_AlgaeSubsystem.c_autoAlgaeWheelsRun(0.5),
    new WaitCommand(0.1),
    m_coralClawSubsystem.c_autoCoralClawWheelsRun(0.5),
    new WaitCommand(0.1),
    m_AlgaeSubsystem.c_autoAlgaeArmsSetResting()
  );

  }

}

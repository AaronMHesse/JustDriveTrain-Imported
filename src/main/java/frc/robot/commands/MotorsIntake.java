package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralClawSubsystem;

public class MotorsIntake extends ParallelCommandGroup {

  public MotorsIntake(CoralClawSubsystem m_coralClawSubsystem, AlgaeSubsystem m_algaeSubsystem) {

    addCommands(
    m_coralClawSubsystem.c_autoCoralWheelsRun(-0.6),
    m_algaeSubsystem.c_autoAlgaeWheelsRun(-0.8)
    );
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ResetEncoders extends ParallelCommandGroup {

  public ResetEncoders(CoralClawSubsystem m_coralClawSubsystem, AlgaeSubsystem m_algaeSubsystem, ElevatorSubsystem m_elevatorSubsystem) {

    addCommands(
      m_coralClawSubsystem.c_resetCoralEncoder(),
      m_algaeSubsystem.c_resetAlgaeEncoder(),
      m_elevatorSubsystem.c_resetElevatorEncoders()
    );
  }
}

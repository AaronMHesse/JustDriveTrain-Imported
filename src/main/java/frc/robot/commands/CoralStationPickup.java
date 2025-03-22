package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class CoralStationPickup extends SequentialCommandGroup {

  public CoralStationPickup(CoralClawSubsystem m_coralClawSubsystem, ElevatorSubsystem m_elevatorSubsystem) {

    addCommands(
      m_elevatorSubsystem.c_elevatorCoralStation(),
      m_coralClawSubsystem.c_coralArmStation(),
      new WaitCommand(0)
    );
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeArms;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeProcessor extends SequentialCommandGroup {
  public AlgaeProcessor(ElevatorSubsystem m_elevator, AlgaeArms m_arm) {
    addCommands(
      new InstantCommand(() -> m_elevator.v_setpoint(0))
        .alongWith(new InstantCommand(() -> m_arm.v_setpoint(66)))
    );
  }
}
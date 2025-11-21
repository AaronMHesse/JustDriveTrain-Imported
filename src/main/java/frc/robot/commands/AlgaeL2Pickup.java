package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeArms;
import frc.robot.subsystems.CTRE_CANdle;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeL2Pickup extends SequentialCommandGroup {

  public AlgaeL2Pickup(ElevatorSubsystem elevator, AlgaeArms algaeArm, CTRE_CANdle m_CANdle) {
    addCommands(
      new InstantCommand(() -> m_CANdle.v_L2())
        .alongWith(new InstantCommand(() -> elevator.v_setpoint(10)))
          .alongWith(new InstantCommand(() -> algaeArm.v_setpoint(32)))
    );
  }
}
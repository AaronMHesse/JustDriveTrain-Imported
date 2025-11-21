package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeArms;
import frc.robot.subsystems.CTRE_CANdle;
import frc.robot.subsystems.ElevatorSubsystem;

public class Barge extends SequentialCommandGroup {

  public Barge(ElevatorSubsystem elevator, AlgaeArms algaeArm, CTRE_CANdle m_CANdle) {
    addCommands(
      new InstantCommand(() -> elevator.v_setpoint(57))
      .alongWith(new InstantCommand(() -> m_CANdle.v_barge())),
        new WaitCommand(1),
          new InstantCommand(() -> algaeArm.v_setpoint(8))
    );
  }
}
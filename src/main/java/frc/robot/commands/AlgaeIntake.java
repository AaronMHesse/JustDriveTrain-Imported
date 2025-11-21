package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeArms;
import frc.robot.subsystems.CTRE_CANdle;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeIntake extends SequentialCommandGroup {

  public AlgaeIntake(ElevatorSubsystem elevator, AlgaeArms algaeArm, CTRE_CANdle m_CANdle) {
    addCommands(
      new InstantCommand(() -> m_CANdle.v_L1())
        .alongWith(new InstantCommand(() -> elevator.v_setpoint(0)))
          .alongWith(new InstantCommand(() -> algaeArm.v_setpoint(75)))
    );
  }
}
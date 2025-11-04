package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeArms;
import frc.robot.subsystems.ConnectorX;
import frc.robot.subsystems.ElevatorSubsystem;

public class Barge extends SequentialCommandGroup {

  public Barge(ElevatorSubsystem elevator, AlgaeArms algaeArm, ConnectorX lights) {
    addCommands(
      new InstantCommand(() -> elevator.v_setpoint(0))
      .alongWith(lights.c_elevator4Lights()),
        new WaitCommand(1.8),
          new InstantCommand(() -> algaeArm.v_setpoint(0))
    );
  }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeArms;
import frc.robot.subsystems.ConnectorX;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeL2Pickup extends SequentialCommandGroup {

  public AlgaeL2Pickup(ElevatorSubsystem elevator, AlgaeArms algaeArm, ConnectorX lights) {
    addCommands(
      lights.c_elevator2Lights()
        .alongWith(new InstantCommand(() -> elevator.v_setpoint(0)))
          .alongWith(new InstantCommand(() -> algaeArm.v_setpoint(0)))
    );
  }
}
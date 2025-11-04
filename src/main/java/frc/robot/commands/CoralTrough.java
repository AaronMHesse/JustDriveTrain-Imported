package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ConnectorX;
import frc.robot.subsystems.CoralArms;
import frc.robot.subsystems.CoralRotator;
import frc.robot.subsystems.ElevatorSubsystem;

public class CoralTrough extends SequentialCommandGroup {

  public CoralTrough(ElevatorSubsystem elevator, CoralArms coralArm, CoralRotator rotator, ConnectorX lights) {
    addCommands(
      lights.c_elevator1Lights()
        .alongWith(new InstantCommand(() -> elevator.v_setpoint(0)))
          .alongWith(new InstantCommand(() -> coralArm.v_setpoint(0)))
            .alongWith(new InstantCommand(() -> rotator.v_claw90Deg()))
    );
  }
}

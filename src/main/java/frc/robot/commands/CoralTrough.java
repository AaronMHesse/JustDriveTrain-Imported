package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CTRE_CANdle;
import frc.robot.subsystems.CoralArms;
import frc.robot.subsystems.CoralRotator;
import frc.robot.subsystems.ElevatorSubsystem;

public class CoralTrough extends SequentialCommandGroup {

  public CoralTrough(ElevatorSubsystem elevator, CoralArms coralArm, CoralRotator rotator, CTRE_CANdle m_CANdle) {
    addCommands(
      new InstantCommand(() -> m_CANdle.v_L1())
        .alongWith(new InstantCommand(() -> elevator.v_setpoint(0)))
          .alongWith(new InstantCommand(() -> coralArm.v_setpoint(37)))
            .alongWith(new InstantCommand(() -> rotator.v_claw90Deg()))
    );
  }
}

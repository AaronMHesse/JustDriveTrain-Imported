package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeArms;

public class AlgaeArmHome extends SequentialCommandGroup {
  public AlgaeArmHome(AlgaeArms m_arm) {
    addCommands(
      new InstantCommand(() -> m_arm.v_setpoint(32))
    );
  }
}
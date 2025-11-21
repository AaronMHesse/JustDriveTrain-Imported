package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralArms;
import frc.robot.subsystems.CoralRotator;

public class CoralArmHome extends SequentialCommandGroup {
  public CoralArmHome(CoralArms m_arm, CoralRotator m_rotator) {
    addCommands(
      new InstantCommand(() -> m_arm.v_setpoint(0))
        .alongWith(new InstantCommand(() -> m_rotator.v_clawHome()))
    );
  }
}
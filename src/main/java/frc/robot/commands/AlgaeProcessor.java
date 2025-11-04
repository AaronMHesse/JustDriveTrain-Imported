package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaeArms;

public class AlgaeProcessor extends InstantCommand {
  public AlgaeProcessor(AlgaeArms algaeArm) {
    algaeArm.v_setpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
}

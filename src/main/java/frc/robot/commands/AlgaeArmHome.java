package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaeArms;

public class AlgaeArmHome extends InstantCommand {

  public AlgaeArmHome(AlgaeArms algaeArm) {
    algaeArm.v_setpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
}

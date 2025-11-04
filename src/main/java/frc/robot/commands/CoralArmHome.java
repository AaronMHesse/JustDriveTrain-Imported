package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralArms;
import frc.robot.subsystems.CoralRotator;

public class CoralArmHome extends InstantCommand {
  public CoralArmHome(CoralArms coralArm, CoralRotator rotator) {
    coralArm.v_setpoint(0);
    rotator.v_clawHome();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}

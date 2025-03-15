package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MyConstants;

public class Blinkin extends SubsystemBase {
  Spark blinkin;
XboxController m_driverController = new XboxController(MyConstants.kDriverControllerPort);
	public Blinkin() {
		blinkin = new Spark(0);
	}

    public void c_lightsNormal() {
		blinkin.set(-0.99);
	}

    public void c_lightSet(double value) {
        blinkin.set(value);
      }

  //When picking up from either side
  public void c_armsPickup() {
    blinkin.set(-0.71);
  }

  //When shooting from either side
  public void c_armsOutput() {
    blinkin.set(-0.31);
  }

  public Command c_autoBlinkinPickup() {
    return new InstantCommand(() -> blinkin.set(-0.71));
  }

  public Command c_autoBlinkinOutput() {
    return new InstantCommand(() -> blinkin.set(-0.31));
  }

  public Command c_autoBlinkinStandard() {
    return new InstantCommand(() -> blinkin.set(-0.41));
  }
}

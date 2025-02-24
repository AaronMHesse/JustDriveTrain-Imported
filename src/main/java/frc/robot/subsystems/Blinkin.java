package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
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

    //When shooting coral out
    public void c_coralOutputBlinkin() {
        blinkin.set(0.71);
    }

    //When picking up coral
    public void c_coralPickupBlinkin() {
        blinkin.set(0.69);
    }

    //When shooting algae
    public void c_algaeOutputBlinkin() {
        blinkin.set(0.81);
    }

    //When picking up algae
    public void c_algaePickupBlinkin() {
        blinkin.set(0.91);
    }

    //When robot is moving slower
    public void c_slowSpeedBlinkin() {
        blinkin.set(0.63);
    }

    //When robot is moving faster
    public void c_fastSpeedBlinkin() {
        blinkin.set(0.89);
    }

    
}

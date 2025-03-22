package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.lumynlabs.devices.ConnectorXAnimate;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class ConnectorX extends SubsystemBase {

private ConnectorXAnimate cXAnimate = new ConnectorXAnimate();

  public ConnectorX() {
    
  }

  


  @Override
  public void periodic() {

  }
}

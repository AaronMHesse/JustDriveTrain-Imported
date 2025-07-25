package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.lumynlabs.devices.ConnectorXAnimate;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class ConnectorX extends SubsystemBase {

public ConnectorXAnimate cXAnimate = new ConnectorXAnimate();

  public ConnectorX() {
    cXAnimate.Connect(Port.kUSB);

    cXAnimate.AddEventHandler((e) -> {
        System.out.printf("Found event type %d", e.type.value);
    });

    cXAnimate.leds.SetGroupAnimationSequence("ALL Lights", "Startup Lights");
  }

//ELEVATOR LIGHTS

public Command c_elevator1Lights() {
  return new InstantCommand(() -> {

    cXAnimate.leds.SetGroupAnimationSequence("L1", "L1");
    cXAnimate.leds.SetGroupAnimationSequence("L2", "OFF");
    cXAnimate.leds.SetGroupAnimationSequence("L3", "OFF");
    cXAnimate.leds.SetGroupAnimationSequence("L4", "OFF");
    }, this);
}

public Command c_elevator2Lights() {
    return new InstantCommand(() -> {

      cXAnimate.leds.SetGroupAnimationSequence("L2 FULL", "L2");
      cXAnimate.leds.SetGroupAnimationSequence("L3", "OFF");
      cXAnimate.leds.SetGroupAnimationSequence("L4", "OFF");
    }, this);
}

public Command c_elevator3Lights() {
    return new InstantCommand(() -> {
    
      cXAnimate.leds.SetGroupAnimationSequence("L3 FULL", "L3");
      cXAnimate.leds.SetGroupAnimationSequence("L4", "OFF");
    }, this);
}

public Command c_elevator4Lights() {
    return new InstantCommand(() -> {

      cXAnimate.leds.SetGroupAnimationSequence("FULL Elevator", "L4");
    }, this);
}

}
// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.lumynlabs.devices.ConnectorXAnimate;
// import com.lumynlabs.domain.led.Animation;

// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj.Joystick.AxisType;
// import edu.wpi.first.wpilibj.SerialPort.Port;

// public class ConnectorX extends SubsystemBase {

// private ConnectorXAnimate cXAnimate = new ConnectorXAnimate();

//   public ConnectorX() {
//     cXAnimate.Connect(Port.kUSB1);
//   }

// public Command c_idleLights() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("Captn Bar", Animation.Chase, new Color(204, 3, 180), Units.Milliseconds.of(40), false, false);
//   });
// }

// //PICKUP <|> OUPUT LIGHTS
// public Command c_intakeLights() {
//   return new InstantCommand(() -> cXAnimate.leds.SetAnimation("Captn Bar", Animation.SineRoll, new Color(26, 177, 0), Units.Milliseconds.of(10), true, false));
// }

// public Command c_outputLights() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("Captn Bar", Animation.SineRoll, new Color(211, 0, 0), Units.Milliseconds.of(10), false, false);
//   });
// }


// //ELEVATOR LIGHTS
// public Command c_elevator1Lights() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("L1 (1)", Animation.SineRoll, new Color(18, 0, 172), Units.Milliseconds.of(200), false, false);
//     cXAnimate.leds.SetAnimation("L1 (2)", Animation.SineRoll, new Color(18, 0, 172), Units.Milliseconds.of(200), false, false);
//   });
// }

// public Command c_elevator2Lights() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("L2 (1)", Animation.SineRoll, new Color(209, 156, 0), Units.Milliseconds.of(200), false, false);
//     cXAnimate.leds.SetAnimation("L2 (2)", Animation.SineRoll, new Color(209, 156, 0), Units.Milliseconds.of(200), false, false);
//   });
// }

// public Command c_elevator3Lights() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("L3 (1)", Animation.SineRoll, new Color(143, 0, 163), Units.Milliseconds.of(200), false, false);
//     cXAnimate.leds.SetAnimation("L3 (2)", Animation.SineRoll, new Color(143, 0, 163), Units.Milliseconds.of(200), false, false);
//   });
// }

// public Command c_elevator4Lights() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("L4 (1)", Animation.SineRoll, new Color(192, 0, 169), Units.Milliseconds.of(200), false, false);
//     cXAnimate.leds.SetAnimation("L4 (2)", Animation.SineRoll, new Color(192, 0, 169), Units.Milliseconds.of(200), false, false);
//   });

// }

//   @Override
//   public void periodic() {

//   }
// }

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.MyConstants;

// import com.lumynlabs.devices.ConnectorXAnimate;
// import com.lumynlabs.domain.led.Animation;

// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.SerialPort.Port;

// public class ConnectorX extends SubsystemBase {

// private ConnectorXAnimate cXAnimate = new ConnectorXAnimate();

// XboxController m_driverController = new XboxController(0);

//   public ConnectorX() {
//     cXAnimate.Connect(Port.kUSB1);
//   }

// //PICKUP <|> OUPUT LIGHTS
// public Command c_intakeLights() {
//   return new InstantCommand(() -> cXAnimate.leds.SetAnimation("Captn Bar", Animation.Chase, new Color(26, 177, 0), Units.Milliseconds.of(1), true, false), this);
// }

// public Command c_outputLights() {
//   return new InstantCommand(() -> {
//     if (MyConstants.kTriggerL >= -0.5 || MyConstants.kTriggerR >= 0.5) {
//     cXAnimate.leds.SetAnimation("Captn Bar", Animation.Chase, new Color(211, 0, 0), Units.Milliseconds.of(1), false, false);
//     } else {
//         cXAnimate.leds.SetAnimation("Captn Bar", Animation.Breathe, new Color(0, 100, 255), Units.Milliseconds.of(10), false, false);
//     }
//   }, this);
// }


// //ELEVATOR LIGHTS

// public Command c_elevatorFill() {
//     return new InstantCommand(() -> {
//     cXAnimate.leds.SetGroupAnimation("L1", Animation.SineRoll, new Color(255, 255, 255), Units.Milliseconds.of(1), false, false);
//     cXAnimate.leds.SetGroupAnimation("L2", Animation.SineRoll, new Color(255, 255, 255), Units.Milliseconds.of(1), false, false);
//     cXAnimate.leds.SetGroupAnimation("L3", Animation.SineRoll, new Color(255, 255, 255), Units.Milliseconds.of(1), false, false);
//     cXAnimate.leds.SetGroupAnimation("L4", Animation.SineRoll, new Color(255, 255, 255), Units.Milliseconds.of(1), false, false);
//     });
// }

// public Command c_elevator1Lights() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetGroupAnimation("L1", Animation.SineRoll, new Color(18, 0, 172), Units.Milliseconds.of(200), false, false);
//   }, this);
// }

// public Command c_elevator2Lights() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetGroupAnimation("L2", Animation.SineRoll, new Color(209, 156, 0), Units.Milliseconds.of(200), false, false);
//   }, this);
// }

// public Command c_elevator3Lights() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetGroupAnimation("L3", Animation.SineRoll, new Color(143, 0, 163), Units.Milliseconds.of(200), false, false);
//   }, this);
// }

// public Command c_elevator4Lights() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetGroupAnimation("L4", Animation.SineRoll, new Color(192, 0, 169), Units.Milliseconds.of(200), false, false);
//   }, this);
// }

// // public Command c_elevatorLightCrtl () {
// //     return new InstantCommand(() -> {
// //         if (ElevatorSubsystem.elevatorPosition <= 41) {
// //             cXAnimate.leds.SetAnimation("L1 (1)", Animation.SineRoll, new Color(18, 0, 172), Units.Milliseconds.of(200), false, false);
// //             cXAnimate.leds.SetAnimation("L1 (2)", Animation.SineRoll, new Color(18, 0, 172), Units.Milliseconds.of(200), false, false);
// //         } else if (ElevatorSubsystem.elevatorPosition <= 83) {
// //             cXAnimate.leds.SetAnimation("L2 (1)", Animation.SineRoll, new Color(209, 156, 0), Units.Milliseconds.of(200), false, false);
// //             cXAnimate.leds.SetAnimation("L2 (2)", Animation.SineRoll, new Color(209, 156, 0), Units.Milliseconds.of(200), false, false);
// //         } else if (ElevatorSubsystem.elevatorPosition <= 189) {
// //             cXAnimate.leds.SetAnimation("L3 (1)", Animation.SineRoll, new Color(143, 0, 163), Units.Milliseconds.of(200), false, false);
// //             cXAnimate.leds.SetAnimation("L3 (2)", Animation.SineRoll, new Color(143, 0, 163), Units.Milliseconds.of(200), false, false);
// //         } else if (ElevatorSubsystem.elevatorPosition >= 190) {
// //             cXAnimate.leds.SetAnimation("L4 (1)", Animation.SineRoll, new Color(192, 0, 169), Units.Milliseconds.of(200), false, false);
// //             cXAnimate.leds.SetAnimation("L4 (2)", Animation.SineRoll, new Color(192, 0, 169), Units.Milliseconds.of(200), false, false);   
// //         }
// //     }, this);
// //     }


//   @Override
//   public void periodic() {
//     // c_elevatorLightCrtl();
//   }
// }

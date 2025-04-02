// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ScheduleCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
//   return new InstantCommand(() -> cXAnimate.leds.SetAnimation("Captn Bar", Animation.SineRoll, new Color(26, 177, 0), Units.Milliseconds.of(1), true, false), this);
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


// public Command LightEmUp () {
//   return new SequentialCommandGroup(
//     c_lightEmUpBar(),
//     c_lightEmUpL1(),
//     new WaitCommand(0.5),
//     c_lightEmUpL2(),
//     new WaitCommand(0.5),
//     c_lightEmUpL3(),
//     new WaitCommand(0.5),
//     c_lightEmUpL4()
//   );
// }


// //DEFAULT CMDS

// public Command c_lightEmUpBar() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("Captn Bar", Animation.RainbowCycle, new Color(255, 255, 255), Units.Milliseconds.of(25), false, false);
//   }, this);
// }

// public Command c_lightEmUpL1() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("L1 (1)", Animation.GrowingBreathe, new Color(50, 255, 0), Units.Milliseconds.of(1), false, true);
//     cXAnimate.leds.SetAnimation("L1 (2)", Animation.GrowingBreathe, new Color(50, 255, 0), Units.Milliseconds.of(1), false, true);
//   }, this);
// }

// public Command c_lightEmUpL2() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("L2 (1)", Animation.GrowingBreathe, new Color(50, 255, 0), Units.Milliseconds.of(1), false, true);
//     cXAnimate.leds.SetAnimation("L2 (2)", Animation.GrowingBreathe, new Color(50, 255, 0), Units.Milliseconds.of(1), false, true);
//   }, this);
// }

// public Command c_lightEmUpL3() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("L3 (1)", Animation.GrowingBreathe, new Color(50, 255, 0), Units.Milliseconds.of(1), false, true);
//     cXAnimate.leds.SetAnimation("L3 (2)", Animation.GrowingBreathe, new Color(50, 255, 0), Units.Milliseconds.of(1), false, true);
//   }, this);
// }

// public Command c_lightEmUpL4() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("L4 (1)", Animation.GrowingBreathe, new Color(50, 255, 0), Units.Milliseconds.of(1), false, true);
//     cXAnimate.leds.SetAnimation("L4 (2)", Animation.GrowingBreathe, new Color(50, 255, 0), Units.Milliseconds.of(1), false, true);
//   }, this);
// }



// //ELEVATOR LIGHTS

// public Command c_elevator1Lights() {
//   return new InstantCommand(() -> {
//       cXAnimate.leds.SetAnimation("L1 (1)", Animation.GrowingBreathe, new Color(18, 0, 172), Units.Milliseconds.of(1), false, true);
//       cXAnimate.leds.SetAnimation("L1 (2)", Animation.GrowingBreathe, new Color(18, 0, 172), Units.Milliseconds.of(1), false, true);
  
//       cXAnimate.leds.SetAnimation("L2 (1)", Animation.FadeOut, new Color(0, 0, 0), Units.Milliseconds.of(1), false, true);
//       cXAnimate.leds.SetAnimation("L2 (2)", Animation.FadeOut, new Color(0, 0, 0), Units.Milliseconds.of(1), false, true);
  
//       cXAnimate.leds.SetAnimation("L3 (1)", Animation.FadeOut, new Color(0, 0, 0), Units.Milliseconds.of(1), false, true);
//       cXAnimate.leds.SetAnimation("L3 (2)", Animation.FadeOut, new Color(0, 0, 0), Units.Milliseconds.of(1), false, true);
  
//       cXAnimate.leds.SetAnimation("L4 (1)", Animation.FadeOut, new Color(0, 0, 0), Units.Milliseconds.of(1), false, true);
//       cXAnimate.leds.SetAnimation("L4 (2)", Animation.FadeOut, new Color(0, 0, 0), Units.Milliseconds.of(1), false, true);
//   }, this);
// }

// public Command c_elevator2Lights() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("L1 (1)", Animation.FadeIn, new Color(209, 156, 0), Units.Milliseconds.of(1), false, true);
//     cXAnimate.leds.SetAnimation("L1 (2)", Animation.FadeIn, new Color(209, 156, 0), Units.Milliseconds.of(1), false, true);

//     cXAnimate.leds.SetAnimation("L2 (1)", Animation.GrowingBreathe, new Color(209, 156, 0), Units.Milliseconds.of(1), false, true);
//     cXAnimate.leds.SetAnimation("L2 (2)", Animation.GrowingBreathe, new Color(209, 156, 0), Units.Milliseconds.of(1), false, true);

//     cXAnimate.leds.SetAnimation("L3 (1)", Animation.FadeOut, new Color(0, 0, 0), Units.Milliseconds.of(1), false, true);
//     cXAnimate.leds.SetAnimation("L3 (2)", Animation.FadeOut, new Color(0, 0, 0), Units.Milliseconds.of(1), false, true);

//     cXAnimate.leds.SetAnimation("L4 (1)", Animation.FadeOut, new Color(0, 0, 0), Units.Milliseconds.of(1), false, true);
//     cXAnimate.leds.SetAnimation("L4 (2)", Animation.FadeOut, new Color(0, 0, 0), Units.Milliseconds.of(1), false, true);
//   }, this);
// }

// public Command c_elevator3Lights() {
//   return new InstantCommand(() -> {
//       cXAnimate.leds.SetAnimation("L1 (1)", Animation.FadeIn, new Color(143, 0, 163), Units.Milliseconds.of(1), false, true);
//       cXAnimate.leds.SetAnimation("L1 (2)", Animation.FadeIn, new Color(143, 0, 163), Units.Milliseconds.of(1), false, true);
  
//       cXAnimate.leds.SetAnimation("L2 (1)", Animation.FadeIn, new Color(143, 0, 163), Units.Milliseconds.of(1), false, true);
//       cXAnimate.leds.SetAnimation("L2 (2)", Animation.FadeIn, new Color(143, 0, 163), Units.Milliseconds.of(1), false, true);
  
//       cXAnimate.leds.SetAnimation("L3 (1)", Animation.GrowingBreathe, new Color(143, 0, 163), Units.Milliseconds.of(1), false, true);
//       cXAnimate.leds.SetAnimation("L3 (2)", Animation.GrowingBreathe, new Color(143, 0, 163), Units.Milliseconds.of(1), false, true);
  
//       cXAnimate.leds.SetAnimation("L4 (1)", Animation.FadeOut, new Color(0, 0, 0), Units.Milliseconds.of(1), false, true);
//       cXAnimate.leds.SetAnimation("L4 (2)", Animation.FadeOut, new Color(0, 0, 0), Units.Milliseconds.of(1), false, true);
//   }, this);
// }

// public Command c_elevator4Lights() {
//   return new InstantCommand(() -> {
//     cXAnimate.leds.SetAnimation("L1 (1)", Animation.FadeIn, new Color(0, 0, 169), Units.Milliseconds.of(1), false, true);
//     cXAnimate.leds.SetAnimation("L1 (2)", Animation.FadeIn, new Color(0, 0, 169), Units.Milliseconds.of(1), false, true);

//     cXAnimate.leds.SetAnimation("L2 (1)", Animation.FadeIn, new Color(0, 0, 169), Units.Milliseconds.of(1), false, true);
//     cXAnimate.leds.SetAnimation("L2 (2)", Animation.FadeIn, new Color(0, 0, 169), Units.Milliseconds.of(1), false, true);

//     cXAnimate.leds.SetAnimation("L3 (1)", Animation.FadeIn, new Color(0, 0, 169), Units.Milliseconds.of(1), false, true);
//     cXAnimate.leds.SetAnimation("L3 (2)", Animation.FadeIn, new Color(0, 0, 169), Units.Milliseconds.of(1), false, true);

//     cXAnimate.leds.SetAnimation("L4 (1)", Animation.GrowingBreathe, new Color(0, 0, 169), Units.Milliseconds.of(1), false, true);
//     cXAnimate.leds.SetAnimation("L4 (2)", Animation.GrowingBreathe, new Color(0, 0, 169), Units.Milliseconds.of(1), false, true);
//   }, this);
// }


//   @Override
//   public void periodic() {

//   }
// }
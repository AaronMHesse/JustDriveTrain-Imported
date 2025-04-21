// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;


// public class Blinkin extends SubsystemBase {
//   Spark blinkin;
// 	public Blinkin() {
// 		blinkin = new Spark(0);
// 	}

//   //When no buttons are pressed
//     public void c_lightsNormal() {
// 		blinkin.set(-0.41);
// 	}

//   //When picking up from either side
//     public void c_armsPickup() {
//       blinkin.set(-0.91);
//   }

//   //When shooting from either side
//     public void c_armsOutput() {
//       blinkin.set(-0.93);
//   }


//     public Command c_autoBlinkinPickup() {
//       return new InstantCommand(() -> blinkin.set(-0.91));
//   }

//     public Command c_autoBlinkinOutput() {
//       return new InstantCommand(() -> blinkin.set(-0.93));
//   }

//     public Command c_autoBlinkinNormal() {
//       return new InstantCommand(() -> blinkin.set(-0.41));
//   }
// }
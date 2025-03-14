// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.MyConstants;
// import frc.robot.subsystems.TempAlgae;
// import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.CoralClawSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.io.IOException;
import java.lang.ModuleLayer.Controller;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;
  private final Blinkin m_blinkin = new Blinkin();
  // public final CoralClawSubsystem m_coralClawSubsystem = new CoralClawSubsystem();
  // public final TempAlgae m_TempAlgae = new TempAlgae();
  // public final AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final SendableChooser<Command> autoChooser;

  //CONTROLLERS SETUP
  XboxController m_driverController = new XboxController(0);
  // XboxController m_operatorController = new XboxController(1);
  GenericHID m_operatorBoard = new GenericHID(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * @throws ParseException 
   * @throws IOException 
   */
  public RobotContainer() throws ParseException, IOException{

    

    // Configure the button bindings

    m_robotDrive = new DriveSubsystem();

    // NamedCommands.registerCommand("Run Coral Output", m_coralClawSubsystem.c_autoCoralWheelsRun(0.5));
    // NamedCommands.registerCommand("Stop Coral Output", m_coralClawSubsystem.c_autoCoralWheelsRun(0));


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser 6175", autoChooser);

    



    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
      
        new RunCommand(

            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController.getLeftY() * 0.60 ), Constants.MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getLeftX() * 0.60 ), Constants.MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX() * 0.8), Constants.MyConstants.kDriveDeadband),
                true, true),
            m_robotDrive));


// DRIVER BUTTONS
// new JoystickButton(m_driverController, MyConstants.kXButton).whileTrue(new RunCommand(() -> m_coralClawSubsystem.c_coralArmRun(0.1), m_coralClawSubsystem));
// new JoystickButton(m_driverController, MyConstants.kXButton).whileFalse(new RunCommand(() -> m_coralClawSubsystem.c_coralArmRun(0), m_coralClawSubsystem));

// new JoystickButton(m_driverController, MyConstants.kBButton).whileTrue(new RunCommand(() -> m_coralClawSubsystem.c_coralArmRun(-0.05), m_coralClawSubsystem));
// new JoystickButton(m_driverController, MyConstants.kBButton).whileFalse(new RunCommand(() -> m_coralClawSubsystem.c_coralArmRun(0), m_coralClawSubsystem));

// new JoystickButton(m_driverController, MyConstants.kYButton).whileTrue(new RunCommand(() -> m_AlgaeSubsystem.c_algaeArmsRun(0.2), m_AlgaeSubsystem));
// new JoystickButton(m_driverController, MyConstants.kYButton).whileFalse(new RunCommand(() -> m_AlgaeSubsystem.c_algaeArmsRun(0), m_AlgaeSubsystem));

// new JoystickButton(m_driverController, MyConstants.kAButton).whileTrue(new RunCommand(() -> m_AlgaeSubsystem.c_algaeArmsRun(-0.15), m_AlgaeSubsystem));
// new JoystickButton(m_driverController, MyConstants.kAButton).whileFalse(new RunCommand(() -> m_AlgaeSubsystem.c_algaeArmsRun(0), m_AlgaeSubsystem));

new JoystickButton(m_driverController, MyConstants.kXButton).onTrue(new RunCommand(() -> m_elevatorSubsystem.c_elevatorDown(), m_elevatorSubsystem));
new JoystickButton(m_driverController, MyConstants.kBButton).onTrue(new RunCommand(() -> m_elevatorSubsystem.c_elevatorUp(), m_elevatorSubsystem));

new JoystickButton(m_driverController, MyConstants.kYButton).whileTrue(new RunCommand(() -> m_elevatorSubsystem.c_elevatorJog(0.1), m_elevatorSubsystem));
new JoystickButton(m_driverController, MyConstants.kYButton).whileFalse(new RunCommand(() -> m_elevatorSubsystem.c_elevatorJog(0), m_elevatorSubsystem));
new JoystickButton(m_driverController, MyConstants.kAButton).whileTrue(new RunCommand(() -> m_elevatorSubsystem.c_elevatorJog(-0.05), m_elevatorSubsystem));
new JoystickButton(m_driverController, MyConstants.kAButton).whileFalse(new RunCommand(() -> m_elevatorSubsystem.c_elevatorJog(0), m_elevatorSubsystem));


    // DRIVER SPEED BUTTON
new JoystickButton(m_driverController, MyConstants.kBumperL).whileTrue(new RunCommand(
  () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController.getLeftY() * 0.20), MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getLeftX() * 0.20), MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX()) * 0.4, MyConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
new JoystickButton(m_driverController, MyConstants.kBumperL).whileFalse(new RunCommand(
  () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController.getLeftY() * 0.60), MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getLeftX() * 0.60), MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX()) * 0.8, MyConstants.kDriveDeadband),
                true, true),
            m_robotDrive));


if (m_driverController.getLeftBumperButtonReleased()) {
  new JoystickButton(m_driverController, MyConstants.kBumperR).whileTrue(new RunCommand(
  () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), MyConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
new JoystickButton(m_driverController, MyConstants.kBumperR).whileFalse(new RunCommand(
  () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController.getLeftY() * 0.60), MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getLeftX() * 0.60), MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX()) * 0.8, MyConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
}


// //D-PAD UP
// new POVButton(m_driverController, 0).whileTrue(new RunCommand(() -> m_coralClawSubsystem.c_coralWheelsRun(0.6), m_coralClawSubsystem));
// new POVButton(m_driverController, 0).whileFalse(new RunCommand(() -> m_coralClawSubsystem.c_coralWheelsRun(0), m_coralClawSubsystem));

// //D-PAD DOWN
// new POVButton(m_driverController, 180).whileTrue(new RunCommand(() -> m_coralClawSubsystem.c_coralWheelsRun(-0.5), m_coralClawSubsystem));
// new POVButton(m_driverController, 180).whileFalse(new RunCommand(() -> m_coralClawSubsystem.c_coralWheelsRun(0), m_coralClawSubsystem));

// //D-PAD RIGHT
// new POVButton(m_driverController, 90).whileTrue(new RunCommand(() -> m_AlgaeSubsystem.c_algaeWheelsRun(1.0), m_AlgaeSubsystem));
// new POVButton(m_driverController, 90).whileFalse(new RunCommand(() -> m_AlgaeSubsystem.c_algaeWheelsRun(0), m_AlgaeSubsystem));

// //D-PAD LEFT
// new POVButton(m_driverController, 270).whileTrue(new RunCommand(() -> m_AlgaeSubsystem.c_algaeWheelsRun(-0.6), m_AlgaeSubsystem));
// new POVButton(m_driverController, 270).whileFalse(new RunCommand(() -> m_AlgaeSubsystem.c_algaeWheelsRun(0), m_AlgaeSubsystem));

// //ALGAE ARM STRAIGHT
// new JoystickButton(m_driverController, 1).onTrue(new RunCommand(() -> m_AlgaeSubsystem.c_algaeArmsSetResting(), m_AlgaeSubsystem));

// //ALGAE ARM RESTING POSITION
// new JoystickButton(m_driverController, 4).onTrue(new RunCommand(() -> m_AlgaeSubsystem.c_algaeArmsSetIntake(), m_AlgaeSubsystem));


//OPERATOR BUTTON BOARD
// new JoystickButton(m_operatorBoard, 1).onTrue(new RunCommand(() -> m_AlgaeSubsystem.c_algaeArmsSetIntake(), m_AlgaeSubsystem));
// new JoystickButton(m_operatorBoard, 2).onTrue(new RunCommand(() -> m_AlgaeSubsystem.c_algaeArmsSetResting(), m_AlgaeSubsystem));
// new JoystickButton(m_driverController, 3).onTrue(new RunCommand(() -> m_coralClawSubsystem.c_coralArmSetIntake(), m_coralClawSubsystem));
// new JoystickButton(m_driverController, 4).onTrue(new RunCommand(() -> m_coralClawSubsystem.c_coralArmSetResting(), m_coralClawSubsystem));


                      //BLINKIN COMMANDS//
new POVButton(m_driverController, 0).whileTrue(new RunCommand(() -> m_blinkin.c_coralOutputBlinkin(), m_blinkin));
new POVButton(m_driverController, 180).whileTrue(new RunCommand(() -> m_blinkin.c_coralPickupBlinkin(), m_blinkin));
new POVButton(m_driverController, 90).whileTrue(new RunCommand(() -> m_blinkin.c_algaeOutputBlinkin(), m_blinkin));
new POVButton(m_driverController, 270).whileTrue(new RunCommand(() -> m_blinkin.c_algaePickupBlinkin(), m_blinkin));
}

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  }

  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  public Command getAutonomousCommand() {

      return autoChooser.getSelected();
  }

  //   // Create config for trajectory
  //   TrajectoryConfig config = new TrajectoryConfig(
  //     Constants.MyConstants.kMaxSpeedMetersPerSecond,
  //     Constants.MyConstants.kAutoMaxAccelerationMetersPerSecondSquared)
  //       // Add kinematics to ensure max speed is actually obeyed
  //       .setKinematics(Constants.MyConstants.kDriveKinematics);

  //   // An example trajectory to follow. All units in meters.
  //   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  //       // Start at the origin facing the +X direction
  //       new Pose2d(0, 0, new Rotation2d(0)),
  //       // Pass through these two interior waypoints, making an 's' curve path
  //       List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  //       // End 3 meters straight ahead of where we started, facing forward
  //       new Pose2d(3, 0, new Rotation2d(0)),
  //       config);

  //   var thetaController = new ProfiledPIDController(
  //     Constants.MyConstants.kPThetaController, 0, 0, Constants.MyConstants.kThetaControllerConstraints);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
  //       exampleTrajectory,
  //       m_robotDrive::getPose, // Functional interface to feed supplier
  //       Constants.MyConstants.kDriveKinematics,

  //       // Position controllers
  //       new PIDController(Constants.MyConstants.kPXController, 0, 0),
  //       new PIDController(Constants.MyConstants.kPYController, 0, 0),
  //       thetaController,
  //       m_robotDrive::setModuleStates,
  //       m_robotDrive);

  //   // Reset odometry to the starting pose of the trajectory.
  //   m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  //   // Run path following command, then stop at the end.
  //   return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  // }
}

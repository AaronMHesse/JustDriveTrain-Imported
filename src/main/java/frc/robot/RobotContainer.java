package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MyConstants;
import frc.robot.subsystems.AlgaeArms;
import frc.robot.subsystems.AlgaeWheels;
import frc.robot.subsystems.ConnectorX;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralArms;
import frc.robot.subsystems.CoralRotator;
import frc.robot.subsystems.CoralWheels;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.io.IOException;


import org.json.simple.parser.ParseException;

import com.fasterxml.jackson.databind.node.IntNode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  
//GIVING SUBSYSTEMS NAMES FOR RUNNING CODE
  public final DriveSubsystem m_robotDrive;
  public final CoralArms m_coralArm = new CoralArms();
  private final CoralWheels m_coralWheels = new CoralWheels();
  private final CoralRotator m_coralRotator = new CoralRotator();
  public final AlgaeArms m_algaeArms = new AlgaeArms();
  private final AlgaeWheels m_algaeWheels = new AlgaeWheels();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
private final ConnectorX m_connectorX = new ConnectorX();
  private final SendableChooser<Command> autoChooser;

  //CONTROLLER SETUP
  CommandXboxController m_driverController = new CommandXboxController(0);
  CommandGenericHID m_operatorBoard = new CommandGenericHID(1);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * @throws ParseException 
   * @throws IOException 
   */
  public RobotContainer() throws ParseException, IOException{

    // Configure the button bindings

    m_robotDrive = new DriveSubsystem();

    //AUTONOMOUS COMMANDS

        //CORAL COMMANDS
    NamedCommands.registerCommand("CoralOutput", m_coralWheels.c_outsideCoralWheelsRun(-0.5));
    NamedCommands.registerCommand("90DegCoralOutput", m_coralWheels.c_coral90DegOutput());
    NamedCommands.registerCommand("CoralPickup", m_coralWheels.c_outsideCoralWheelsRun(0.75));
    NamedCommands.registerCommand("StopCoralWheels", m_coralWheels.c_allWheelsRun(0));
    NamedCommands.registerCommand("CoralRotateHome", m_coralRotator.c_clawHome());
    NamedCommands.registerCommand("CoralRotate90Deg", m_coralRotator.c_claw90Deg());
    NamedCommands.registerCommand("CoralResting", m_coralArm.c_coralArmResting());
    NamedCommands.registerCommand("CoralHoldResting", m_coralArm.c_autoCoralArmHoldResting());
    NamedCommands.registerCommand("CoralTrough", m_coralArm.c_coralTrough());
    NamedCommands.registerCommand("CoralStationPickup", m_coralArm.c_coralArmStation());
    NamedCommands.registerCommand("CoralIntake", m_coralArm.c_autoCoralArmIntake());
    NamedCommands.registerCommand("CoralReef", m_coralArm.c_coralReef());
    
        // ALGAE COMMANDS
    NamedCommands.registerCommand("AlgaeOutput", m_algaeWheels.c_autoAlgaeWheelsRun(0.4));
    NamedCommands.registerCommand("AlgaePickup", m_algaeWheels.c_autoAlgaeWheelsRun(-0.8));
    NamedCommands.registerCommand("StopAlgaeWheels", m_algaeWheels.c_autoAlgaeWheelsRun(0));
    // NamedCommands.registerCommand("AlgaeResting", m_algaeArms.c_algaeArmsResting());
    // NamedCommands.registerCommand("AlgaeHoldResting", m_algaeArms.c_algaeArmsResting());
    // NamedCommands.registerCommand("AlgaeProcessor", m_algaeArms.c_algaeArmsProcessor());
    // NamedCommands.registerCommand("AlgaeIntake", m_algaeArms.c_algaeArmsIntake());

        //ELEVATOR COMMANDS
    NamedCommands.registerCommand("ElevatorDown", m_elevatorSubsystem.c_autoElevatorDown());
    NamedCommands.registerCommand("ElevatorStation", m_elevatorSubsystem.c_elevatorCoralStation());
    NamedCommands.registerCommand("ElevatorLevel2", m_elevatorSubsystem.c_elevatorL2());
    NamedCommands.registerCommand("ElevatorLevel3", m_elevatorSubsystem.c_elevatorL3());
    NamedCommands.registerCommand("ElevatorLevel4", m_elevatorSubsystem.c_elevatorL4());
    
    NamedCommands.registerCommand("LightsL1", m_connectorX.c_elevator1Lights());
    NamedCommands.registerCommand("LightsL2", m_connectorX.c_elevator2Lights());
    NamedCommands.registerCommand("LightsL3", m_connectorX.c_elevator3Lights());
    NamedCommands.registerCommand("LightsL4", m_connectorX.c_elevator4Lights());
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser 6175", autoChooser);

    configureBindings();
  }

    private void configureBindings() {

    m_robotDrive.setDefaultCommand(
        new RunCommand(

            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController.getLeftY() * 0.6 ), Constants.MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getLeftX() * 0.6 ), Constants.MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX() * 0.8), Constants.MyConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

// m_algaeWheels.setDefaultCommand(new RunCommand(() -> m_algaeWheels.c_algaeWheelsOutput(MyConstants.kTriggerL, 0.45), m_algaeWheels));
// m_coralWheels.setDefaultCommand(new RunCommand(() -> m_coralWheels.v_coralOutput(MyConstants.kTriggerR), m_coralWheels));

//DRIVER BUTTONS

    m_driverController.a().onTrue(m_algaeArms.run(() -> m_algaeArms.v_setPos(10)));

    m_driverController.leftTrigger(0.5)
    .onTrue(new RunCommand(() -> m_algaeWheels.c_algaeWheelsOutput(0.30, 0.45), m_algaeArms))
    .onFalse(new RunCommand(() -> m_algaeWheels.c_algaeWheelsOutput(0, 0), m_algaeArms));

    m_driverController.rightTrigger(0.5)
    .onTrue(new RunCommand(() -> m_coralWheels.v_coralOutput(0.7, 0.7, 1, 0.5), m_coralWheels))
    .onFalse(new RunCommand(() -> m_coralWheels.v_coralOutput(0, 0, 0, 0), m_coralWheels));

    //ZERO GYRO                             OPTION BUTTON
    m_driverController.start().onTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));



   //RESET ENCODERS                         SHARE BUTTON
    m_driverController.back().onTrue(m_elevatorSubsystem.c_resetElevatorEncoders());



    //MANUAL CONTROLS

        // INTAKE/OUPUT CONTROLS
    m_driverController.leftBumper().whileTrue(m_algaeWheels.run(() -> m_algaeWheels.c_algaeWheelsRun(-0.8)));
    // new JoystickButton(m_driverController, MyConstants.kBumperL).whileTrue(new RunCommand(() -> m_algaeWheels.c_algaeWheelsRun(-0.8), m_algaeWheels));
    // new JoystickButton(m_driverController, MyConstants.kBumperL).whileFalse(new RunCommand(() -> m_algaeWheels.c_algaeWheelsOutput(MyConstants.kTriggerL, 0.45), m_algaeWheels));

    m_driverController.rightBumper()
    .whileTrue(new InstantCommand(() -> m_coralWheels.v_coralIntake(0.75)))
    .whileFalse(new InstantCommand(() -> m_coralWheels.v_coralIntake(0)));
    // new JoystickButton(m_driverController, MyConstants.kBumperR).whileTrue(new RunCommand(() -> m_coralWheels.v_coralIntake(), m_coralWheels));
    // new JoystickButton(m_driverController, MyConstants.kBumperR).whileFalse(new RunCommand(() -> m_coralWheels.c_coralWheelsOutput(MyConstants.kTriggerR, 0.4), m_coralWheels));


        //ALL MANIPULATORS

                                        //RIGHT STICK DOWN
    m_driverController.rightStick().onTrue
    (new InstantCommand(() -> m_connectorX.c_elevator1Lights())
    .alongWith(new InstantCommand(() -> m_elevatorSubsystem.c_autoElevatorDown()))
    .alongWith(new InstantCommand(() -> m_coralArm.c_coralArmResting()))
    .alongWith(new InstantCommand(() -> m_coralRotator.c_clawHome())));
    // .alongWith(new InstantCommand(() -> m_algaeArms.c_algaeArmsResting)));
    // new JoystickButton(m_driverController, 10).onTrue(new ParallelCommandGroup(
    //     m_connectorX.c_elevator1Lights(),
    //     m_elevatorSubsystem.c_autoElevatorDown(),
    //     m_coralArm.c_coralArmResting(),
    //     // m_algaeArms.c_algaeArmsResting(),
    //     m_coralRotator.c_clawHome()   
    // ));

        //ALGAE ARMS
    // new JoystickButton(m_driverController, MyConstants.kYButton).whileTrue(new RunCommand(() -> m_algaeArms.c_algaeArmsJog(0.4), m_algaeArms));
    // new JoystickButton(m_driverController, MyConstants.kYButton).whileFalse(new RunCommand(() -> m_algaeArms.c_algaeArmsJog(0), m_algaeArms));
    // new JoystickButton(m_driverController, MyConstants.kAButton).whileTrue(new RunCommand(() -> m_algaeArms.c_algaeArmsJog(-0.4), m_algaeArms));
    // new JoystickButton(m_driverController, MyConstants.kAButton).whileFalse(new RunCommand(() -> m_algaeArms.c_algaeArmsJog(0), m_algaeArms));

    m_driverController.a().onTrue(new InstantCommand(() -> m_algaeArms.v_setPos(10)));
    m_driverController.y().onTrue(new InstantCommand(() -> m_coralRotator.v_claw90Deg()));
    // new JoystickButton(m_driverController, MyConstants.kAButton).onTrue(new RunCommand(() -> m_algaeArms.v_setPos(10), m_algaeArms));
    // new JoystickButton(m_driverController, MyConstants.kYButton).onTrue(new RunCommand(() -> m_coralRotator.v_claw90Deg(), m_coralRotator));

        //CORAL ARM
    m_driverController.x()
    .whileTrue(new InstantCommand(() -> m_coralArm.v_coralArmJog(-0.3)))
    .whileFalse(new InstantCommand(() -> m_coralArm.v_coralArmJog(0)));
    m_driverController.b()
    .whileTrue(new InstantCommand(() -> m_coralArm.v_coralArmJog(0.3)))
    .whileFalse(new InstantCommand(() -> m_coralArm.v_coralArmJog(0)));
    // new JoystickButton(m_driverController, MyConstants.kXButton).whileTrue(new RunCommand(() -> m_coralArm.v_coralArmJog(-0.3), m_coralArm));
    // new JoystickButton(m_driverController, MyConstants.kXButton).whileFalse(new RunCommand(() -> m_coralArm.v_coralArmJog(0), m_coralArm));
    // new JoystickButton(m_driverController, MyConstants.kBButton).whileTrue(new RunCommand(() -> m_coralArm.v_coralArmJog(0.3), m_coralArm));
    // new JoystickButton(m_driverController, MyConstants.kBButton).whileFalse(new RunCommand(() -> m_coralArm.v_coralArmJog(0), m_coralArm));

    // MANUAL ELEVATOR

        //D-PAD UP
    m_driverController.povUp()
        .whileTrue(new InstantCommand(() -> m_elevatorSubsystem.c_elevatorJog(0.45)))
        .whileFalse(new InstantCommand(() -> m_elevatorSubsystem.c_elevatorJog(0)));
    // new POVButton(m_driverController, 0).whileTrue(new RunCommand(() -> m_elevatorSubsystem.c_elevatorJog(0.45), m_elevatorSubsystem));
    // new POVButton(m_driverController, 0).whileFalse(new RunCommand(() -> m_elevatorSubsystem.c_elevatorJog(0), m_elevatorSubsystem));

        //D-PAD DOWN
    m_driverController.povDown()
        .whileTrue(new InstantCommand(() -> m_elevatorSubsystem.c_elevatorJog(-0.45)))
        .whileFalse(new InstantCommand(() -> m_elevatorSubsystem.c_elevatorJog(0)));
    // new POVButton(m_driverController, 180).whileTrue(new RunCommand(() -> m_elevatorSubsystem.c_elevatorJog(-0.45), m_elevatorSubsystem));
    // new POVButton(m_driverController, 180).whileFalse(new RunCommand(() -> m_elevatorSubsystem.c_elevatorJog(0), m_elevatorSubsystem));



    // OPERATOR BUTTON BOARD

        // ELEVATOR
    m_operatorBoard.button(1).onTrue
    (new InstantCommand(() -> m_connectorX.c_elevator4Lights())
        .alongWith(new InstantCommand(() -> m_elevatorSubsystem.c_elevatorL4()))
            .alongWith(new InstantCommand(() -> m_coralArm.c_coralReef()))
                .alongWith(new InstantCommand(() -> m_coralRotator.v_clawHome())));
        // new JoystickButton(m_operatorBoard, 1).onTrue(new ParallelCommandGroup(
        //     m_connectorX.c_elevator4Lights(),
        //     m_elevatorSubsystem.c_elevatorL4(),
        //     m_coralArm.c_coralReef(),
        //     m_coralRotator.c_clawHome()
        // ));
    m_operatorBoard.button(4).onTrue
    (new InstantCommand(() -> m_connectorX.c_elevator3Lights())
        .alongWith(new InstantCommand(() -> m_elevatorSubsystem.c_elevatorL3()))
            .alongWith(new InstantCommand(() -> m_coralArm.c_coralReef()))
                .alongWith(new InstantCommand(() -> m_coralRotator.c_clawHome())));
        // new JoystickButton(m_operatorBoard, 4).onTrue(new ParallelCommandGroup(
        //     m_connectorX.c_elevator3Lights(),
        //     m_elevatorSubsystem.c_elevatorL3(),
        //     m_coralArm.c_coralReef(),
        //     m_coralRotator.c_clawHome()
        // ));
    m_operatorBoard.button(7).onTrue
    (new InstantCommand(() -> m_connectorX.c_elevator2Lights())
        .alongWith(new InstantCommand(() -> m_elevatorSubsystem.c_elevatorL2()))
            .alongWith(new InstantCommand(() -> m_coralArm.c_coralReef()))
                .alongWith(new InstantCommand(() -> m_coralRotator.c_clawHome())));
        // new JoystickButton(m_operatorBoard, 7).onTrue(new ParallelCommandGroup(
        //     m_connectorX.c_elevator2Lights(),
        //     m_elevatorSubsystem.c_elevatorL2(),
        //     m_coralArm.c_coralReef(),
        //     m_coralRotator.c_clawHome()
        // ));

    
            // ALGAE CLAW
    m_operatorBoard.button(2).onTrue
    (new InstantCommand(() -> m_connectorX.c_elevator4Lights())
    .alongWith(new InstantCommand(() -> m_elevatorSubsystem.c_elevatorBarge()))
    .withTimeout(1.8));
    // .andThen(new InstantCommand(() -> m_algaeArms.c_algaeBarge)));
        // new JoystickButton(m_operatorBoard, 2).onTrue(new SequentialCommandGroup(
        //     m_connectorX.c_elevator4Lights(),
        //     m_elevatorSubsystem.c_elevatorBarge(),
        //     new WaitCommand(1.8)
        //     // m_algaeArms.c_algaeBarge()
        // ));
    // m_operatorBoard.button(5).onTrue(new InstantCommand(() -> m_algaeArms.v_algaeArmsResting()));
    // m_operatorBoard.button(8).onTrue(new InstantCommand(() -> m_algaeArms.v_algaeArmsProcessor()));
        // new JoystickButton(m_operatorBoard, 5).onTrue(new RunCommand(() -> m_algaeArms.v_algaeArmsResting(), m_algaeArms));
        // new JoystickButton(m_operatorBoard, 8).onTrue(new RunCommand(() -> m_algaeArms.v_algaeArmsProcessor(), m_algaeArms));
    m_operatorBoard.button(11).onTrue
    (new InstantCommand(() -> m_connectorX.c_elevator1Lights())
        .alongWith(new InstantCommand(() -> m_elevatorSubsystem.c_autoElevatorDown())));
            // .alongWith(new InstantCommand(() -> m_algaeArms.c_algaeArmsIntake())));
        // new JoystickButton(m_operatorBoard, 11).onTrue(new ParallelCommandGroup(
        //     m_connectorX.c_elevator1Lights(),
        //     m_elevatorSubsystem.c_autoElevatorDown()
        //     // m_algaeArms.c_algaeArmsIntake()
        // ));
    
            // CORAL CLAW
    m_operatorBoard.button(3).onTrue
    (new InstantCommand(() -> m_connectorX.c_elevator2Lights())
        .alongWith(new InstantCommand(() -> m_elevatorSubsystem.c_elevatorCoralStation()))
            .alongWith(new InstantCommand(() -> m_coralArm.c_coralArmStation()))
                .alongWith(new InstantCommand(() -> m_coralRotator.c_claw90Deg())));
    m_operatorBoard.button(6).onTrue(new InstantCommand(() -> m_coralArm.v_coralArmHoldResting()));
        // new JoystickButton(m_operatorBoard, 6).onTrue(new RunCommand(() -> m_coralArm.v_coralArmHoldResting(), m_coralArm));
        // new JoystickButton(m_operatorBoard, 3).onTrue(new ParallelCommandGroup(
        //     m_connectorX.c_elevator2Lights(),
        //     m_elevatorSubsystem.c_elevatorCoralStation(),
        //     m_coralArm.c_coralArmStation(),
        //     m_coralRotator.c_claw90Deg()
        // ));
    m_operatorBoard.button(9).onTrue
    (new InstantCommand(() -> m_connectorX.c_elevator1Lights())
        .alongWith(new InstantCommand(() -> m_coralArm.c_coralTrough()))
            .alongWith(new InstantCommand(() -> m_coralRotator.c_claw90Deg()))
                .alongWith(new InstantCommand(() -> m_elevatorSubsystem.c_autoElevatorDown())));
    m_operatorBoard.button(12).onTrue
    (new InstantCommand(() -> m_connectorX.c_elevator1Lights())
        .alongWith(new InstantCommand(() -> m_elevatorSubsystem.c_autoElevatorDown()))
            .alongWith(new InstantCommand(() -> m_coralArm.c_autoCoralArmIntake()))
                .alongWith(new InstantCommand(() -> m_coralRotator.c_claw90Deg())));
        // new JoystickButton(m_operatorBoard, 9).onTrue(new ParallelCommandGroup(
        //     m_connectorX.c_elevator1Lights(),
        //     m_coralArm.c_coralTrough(),
        //     m_coralRotator.c_claw90Deg(),
        //     m_elevatorSubsystem.c_autoElevatorDown()
        // ));
        // new JoystickButton(m_operatorBoard, 12).onTrue(new ParallelCommandGroup(
        //     m_connectorX.c_elevator1Lights(),
        //     m_elevatorSubsystem.c_autoElevatorDown(),
        //     m_coralArm.c_autoCoralArmIntake(),
        //     m_coralRotator.c_claw90Deg()
        // ));
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


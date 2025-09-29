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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.io.IOException;


import org.json.simple.parser.ParseException;

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
  XboxController m_driverController = new XboxController(0);
  GenericHID m_operatorBoard = new GenericHID(1);


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
    NamedCommands.registerCommand("CoralPickup", m_coralWheels.c_outsideCoralWheelsRun(-0.5));
    NamedCommands.registerCommand("StopCoralWheels", m_coralWheels.c_outsideCoralWheelsRun(0));
    NamedCommands.registerCommand("CoralRotateHome", m_coralRotator.c_clawHome());
    NamedCommands.registerCommand("CoralRotate90Deg", m_coralRotator.c_claw90Deg());
    NamedCommands.registerCommand("CoralResting", m_coralArm.c_coralArmResting());
    NamedCommands.registerCommand("CoralHoldResting", m_coralArm.c_autoCoralArmHoldResting());
    NamedCommands.registerCommand("CoralTrough", m_coralArm.c_coralTrough());
    NamedCommands.registerCommand("CoralStationPickup", m_coralArm.c_coralArmStation());
    NamedCommands.registerCommand("CoralIntake", m_coralArm.c_autoCoralArmIntake());
    NamedCommands.registerCommand("CoralL4", m_coralArm.c_coralL4());
    NamedCommands.registerCommand("CoralL2", m_coralArm.c_coralL2());
    
        // ALGAE COMMANDS
    NamedCommands.registerCommand("AlgaeOutput", m_algaeWheels.c_autoAlgaeWheelsRun(0.4));
    NamedCommands.registerCommand("AlgaePickup", m_algaeWheels.c_autoAlgaeWheelsRun(-0.8));
    NamedCommands.registerCommand("StopAlgaeWheels", m_algaeWheels.c_autoAlgaeWheelsRun(0));
    NamedCommands.registerCommand("AlgaeResting", m_algaeArms.c_algaeArmsResting());
    NamedCommands.registerCommand("AlgaeHoldResting", m_algaeArms.c_algaeArmsResting());
    NamedCommands.registerCommand("AlgaeProcessor", m_algaeArms.c_algaeArmsProcessor());
    NamedCommands.registerCommand("AlgaeIntake", m_algaeArms.c_algaeArmsIntake());

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

    

    m_robotDrive.setDefaultCommand(
        new RunCommand(

            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController.getLeftY() * 0.6 ), Constants.MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getLeftX() * 0.6 ), Constants.MyConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX() * 0.8), Constants.MyConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

m_algaeWheels.setDefaultCommand(new RunCommand(() -> m_algaeWheels.c_algaeWheelsOutput(MyConstants.kTriggerL, 0.45), m_algaeWheels));
m_coralWheels.setDefaultCommand(new RunCommand(() -> m_coralWheels.v_coralOutput(MyConstants.kTriggerR), m_coralWheels));

//DRIVER BUTTONS

    //ZERO GYRO                             OPTION BUTTON
    new JoystickButton(m_driverController, 8).onTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));



   //RESET ENCODERS                         SHARE BUTTON
    new JoystickButton(m_driverController, 7).onTrue(new ParallelCommandGroup(
        m_elevatorSubsystem.c_resetElevatorEncoders(),
        m_coralArm.c_resetCoralEncoder(),
        m_coralRotator.c_resetRotator()
    ));



    //MANUAL CONTROLS

        // INTAKE/OUPUT CONTROLS
    new JoystickButton(m_driverController, MyConstants.kBumperL).whileTrue(new RunCommand(() -> m_algaeWheels.c_algaeWheelsRun(-0.8), m_algaeWheels));
    // new JoystickButton(m_driverController, MyConstants.kBumperL).whileFalse(new RunCommand(() -> m_algaeWheels.c_algaeWheelsOutput(MyConstants.kTriggerL, 0.45), m_algaeWheels));

    new JoystickButton(m_driverController, MyConstants.kBumperR).whileTrue(new RunCommand(() -> m_coralWheels.v_coralIntake(), m_coralWheels));
    // new JoystickButton(m_driverController, MyConstants.kBumperR).whileFalse(new RunCommand(() -> m_coralWheels.c_coralWheelsOutput(MyConstants.kTriggerR, 0.4), m_coralWheels));


        //ALL MANIPULATORS

                                        //RIGHT STICK DOWN
    new JoystickButton(m_driverController, 10).onTrue(new ParallelCommandGroup(
        m_connectorX.c_elevator1Lights(),
        m_elevatorSubsystem.c_autoElevatorDown(),
        m_coralArm.c_coralArmResting(),
        m_algaeArms.c_algaeArmsResting(),
        m_coralRotator.c_clawHome()   
    ));

        //ALGAE ARMS
    new JoystickButton(m_driverController, MyConstants.kYButton).whileTrue(new RunCommand(() -> m_algaeArms.c_algaeArmsJog(0.4), m_algaeArms));
    new JoystickButton(m_driverController, MyConstants.kYButton).whileFalse(new RunCommand(() -> m_algaeArms.c_algaeArmsJog(0), m_algaeArms));
    new JoystickButton(m_driverController, MyConstants.kAButton).whileTrue(new RunCommand(() -> m_algaeArms.c_algaeArmsJog(-0.4), m_algaeArms));
    new JoystickButton(m_driverController, MyConstants.kAButton).whileFalse(new RunCommand(() -> m_algaeArms.c_algaeArmsJog(0), m_algaeArms));

    // new JoystickButton(m_driverController, MyConstants.kAButton).onTrue(new RunCommand(() -> m_coralRotator.v_clawHome(), m_coralRotator));
    // new JoystickButton(m_driverController, MyConstants.kYButton).onTrue(new RunCommand(() -> m_coralRotator.v_claw90Deg(), m_coralRotator));

        //CORAL ARM
    new JoystickButton(m_driverController, MyConstants.kXButton).whileTrue(new RunCommand(() -> m_coralArm.v_coralArmJog(-0.3), m_coralArm));
    new JoystickButton(m_driverController, MyConstants.kXButton).whileFalse(new RunCommand(() -> m_coralArm.v_coralArmJog(0), m_coralArm));
    new JoystickButton(m_driverController, MyConstants.kBButton).whileTrue(new RunCommand(() -> m_coralArm.v_coralArmJog(0.3), m_coralArm));
    new JoystickButton(m_driverController, MyConstants.kBButton).whileFalse(new RunCommand(() -> m_coralArm.v_coralArmJog(0), m_coralArm));

    // MANUAL ELEVATOR

        //D-PAD UP
    new POVButton(m_driverController, 0).whileTrue(new RunCommand(() -> m_elevatorSubsystem.c_elevatorJog(0.3), m_elevatorSubsystem));
    new POVButton(m_driverController, 0).whileFalse(new RunCommand(() -> m_elevatorSubsystem.c_elevatorJog(0), m_elevatorSubsystem));

        //D-PAD DOWN
    new POVButton(m_driverController, 180).whileTrue(new RunCommand(() -> m_elevatorSubsystem.c_elevatorJog(-0.3), m_elevatorSubsystem));
    new POVButton(m_driverController, 180).whileFalse(new RunCommand(() -> m_elevatorSubsystem.c_elevatorJog(0), m_elevatorSubsystem));



    // OPERATOR BUTTON BOARD

        // ELEVATOR
        new JoystickButton(m_operatorBoard, 1).onTrue(new ParallelCommandGroup(
            m_connectorX.c_elevator4Lights(),
            m_elevatorSubsystem.c_elevatorL4(),
            m_coralArm.c_coralL2(),
            m_coralRotator.c_clawHome()
        ));
        new JoystickButton(m_operatorBoard, 4).onTrue(new ParallelCommandGroup(
            m_connectorX.c_elevator3Lights(),
            m_elevatorSubsystem.c_elevatorL3(),
            m_coralArm.c_coralL2(),
            m_coralRotator.c_clawHome()
        ));
        new JoystickButton(m_operatorBoard, 7).onTrue(new ParallelCommandGroup(
            m_connectorX.c_elevator2Lights(),
            m_elevatorSubsystem.c_elevatorL2(),
            m_coralArm.c_coralL2(),
            m_coralRotator.c_clawHome()
        ));

    
            // ALGAE CLAW
        new JoystickButton(m_operatorBoard, 2).onTrue(new SequentialCommandGroup(
            m_connectorX.c_elevator4Lights(),
            m_elevatorSubsystem.c_elevatorBarge(),
            new WaitCommand(1.8),
            m_algaeArms.c_algaeBarge()
        ));
        new JoystickButton(m_operatorBoard, 5).onTrue(new RunCommand(() -> m_algaeArms.v_algaeArmsResting(), m_algaeArms));
        new JoystickButton(m_operatorBoard, 8).onTrue(new RunCommand(() -> m_algaeArms.v_algaeArmsProcessor(), m_algaeArms));
        new JoystickButton(m_operatorBoard, 11).onTrue(new ParallelCommandGroup(
            m_connectorX.c_elevator1Lights(),
            m_elevatorSubsystem.c_autoElevatorDown(),
            m_algaeArms.c_algaeArmsIntake()
        ));
    
            // CORAL CLAW
        new JoystickButton(m_operatorBoard, 6).onTrue(new RunCommand(() -> m_coralArm.v_coralArmHoldResting(), m_coralArm));
        new JoystickButton(m_operatorBoard, 3).onTrue(new ParallelCommandGroup(
            m_connectorX.c_elevator2Lights(),
            m_elevatorSubsystem.c_elevatorCoralStation(),
            m_coralArm.c_coralArmStation(),
            m_coralRotator.c_claw90Deg()
        ));
        new JoystickButton(m_operatorBoard, 9).onTrue(new ParallelCommandGroup(
            m_connectorX.c_elevator1Lights(),
            m_coralArm.c_coralTrough(),
            m_coralRotator.c_claw90Deg(),
            m_elevatorSubsystem.c_autoElevatorDown()
        ));
        new JoystickButton(m_operatorBoard, 12).onTrue(new ParallelCommandGroup(
            m_connectorX.c_elevator1Lights(),
            m_elevatorSubsystem.c_autoElevatorDown(),
            m_coralArm.c_autoCoralArmIntake(),
            m_coralRotator.c_claw90Deg()
        ));

        // new JoystickButton(m_operatorBoard, 10).whileTrue(new RunCommand(() -> m_algaeWheels.c_algaeWheelsRun(1.5), m_algaeWheels));
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


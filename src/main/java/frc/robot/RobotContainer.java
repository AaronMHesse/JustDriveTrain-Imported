package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AlgaeArmHome;
import frc.robot.commands.AlgaeIntake;
import frc.robot.commands.AlgaeProcessor;
import frc.robot.commands.Barge;
import frc.robot.commands.CoralArmHome;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralL2;
import frc.robot.commands.CoralL3;
import frc.robot.commands.CoralL4;
import frc.robot.commands.CoralStation;
import frc.robot.commands.CoralTrough;
import frc.robot.commands.HomeAllManipulators;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  
//Subsystem Setup
  public final DriveSubsystem m_robotDrive;
  public final CoralArms m_coralArm = new CoralArms();
  private final CoralWheels m_coralWheels = new CoralWheels();
  private final CoralRotator m_coralRotator = new CoralRotator();
  public final AlgaeArms m_algaeArms = new AlgaeArms();
  private final AlgaeWheels m_algaeWheels = new AlgaeWheels();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
private final ConnectorX m_connectorX = new ConnectorX();
  private final SendableChooser<Command> autoChooser;

  //Controllers
  CommandXboxController m_driverController = new CommandXboxController(0);
  CommandGenericHID m_operatorBoard = new CommandGenericHID(1);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * @throws ParseException 
   * @throws IOException 
   */
  public RobotContainer() throws ParseException, IOException{

    m_robotDrive = new DriveSubsystem();

    //Auto Cmds

        //CORAL COMMANDS
    NamedCommands.registerCommand("CoralOutput", m_coralWheels.c_outsideCoralWheelsRun(-0.5));
    NamedCommands.registerCommand("90DegCoralOutput", m_coralWheels.c_coral90DegOutput());
    NamedCommands.registerCommand("CoralPickup", m_coralWheels.c_outsideCoralWheelsRun(0.75));
    NamedCommands.registerCommand("StopCoralWheels", m_coralWheels.c_allWheelsRun(0));
    NamedCommands.registerCommand("CoralRotateHome", m_coralRotator.c_clawHome());
    NamedCommands.registerCommand("CoralRotate90Deg", m_coralRotator.c_claw90Deg());
    // NamedCommands.registerCommand("CoralResting", m_coralArm.c_coralArmResting());
    // NamedCommands.registerCommand("CoralHoldResting", m_coralArm.c_autoCoralArmHoldResting());
    // NamedCommands.registerCommand("CoralTrough", m_coralArm.c_coralTrough());
    // NamedCommands.registerCommand("CoralStationPickup", m_coralArm.c_coralArmStation());
    // NamedCommands.registerCommand("CoralIntake", m_coralArm.c_autoCoralArmIntake());
    // NamedCommands.registerCommand("CoralReef", m_coralArm.c_coralReef());
    
        // ALGAE COMMANDS
    NamedCommands.registerCommand("AlgaeOutput", m_algaeWheels.c_autoAlgaeWheelsRun(0.4));
    NamedCommands.registerCommand("AlgaePickup", m_algaeWheels.c_autoAlgaeWheelsRun(-0.8));
    NamedCommands.registerCommand("StopAlgaeWheels", m_algaeWheels.c_autoAlgaeWheelsRun(0));
    // NamedCommands.registerCommand("AlgaeResting", m_algaeArms.c_algaeArmsResting());
    // NamedCommands.registerCommand("AlgaeHoldResting", m_algaeArms.c_algaeArmsResting());
    // NamedCommands.registerCommand("AlgaeProcessor", m_algaeArms.c_algaeArmsProcessor());
    // NamedCommands.registerCommand("AlgaeIntake", m_algaeArms.c_algaeArmsIntake());

        //ELEVATOR COMMANDS
    // NamedCommands.registerCommand("ElevatorDown", m_elevatorSubsystem.c_autoElevatorDown());
    // NamedCommands.registerCommand("ElevatorStation", m_elevatorSubsystem.c_elevatorCoralStation());
    // NamedCommands.registerCommand("ElevatorLevel2", m_elevatorSubsystem.c_elevatorL2());
    // NamedCommands.registerCommand("ElevatorLevel3", m_elevatorSubsystem.c_elevatorL3());
    // NamedCommands.registerCommand("ElevatorLevel4", m_elevatorSubsystem.c_elevatorL4());
    
    NamedCommands.registerCommand("LightsL1", m_connectorX.c_elevator1Lights());
    NamedCommands.registerCommand("LightsL2", m_connectorX.c_elevator2Lights());
    NamedCommands.registerCommand("LightsL3", m_connectorX.c_elevator3Lights());
    NamedCommands.registerCommand("LightsL4", m_connectorX.c_elevator4Lights());
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

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

//DRIVER BUTTONS

    //ZERO GYRO                             OPTION BUTTON
    m_driverController.start().onTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

   //RESET ENCODERS                         SHARE BUTTON
    m_driverController.back().onTrue(m_elevatorSubsystem.c_resetElevatorEncoders());


    //MANUAL CONTROLS

        // INTAKE/OUPUT CONTROLS
    m_driverController.leftBumper()
        .whileTrue(new InstantCommand(() -> m_algaeWheels.c_algaeWheelsRun(-0.8)))
        .onFalse(new InstantCommand(() -> m_algaeWheels.c_algaeWheelsRun(0)));

    m_driverController.rightBumper()
        .whileTrue(new RunCommand(() -> m_coralWheels.v_coralOutput(0, 0, 0, 0.65), m_coralWheels))
        .onFalse(new InstantCommand(() -> m_coralWheels.v_coralOutput(0, 0, 0, 0)));

        m_driverController.leftTrigger(0.5)
        .whileTrue(new InstantCommand(() -> m_algaeWheels.c_algaeWheelsOutput(0.30, 0.45)))
        .onFalse(new InstantCommand(() -> m_algaeWheels.c_algaeWheelsOutput(0, 0)));

    m_driverController.rightTrigger(0.5)
        .whileTrue(new RunCommand(() -> m_coralWheels.v_coralOutput(0.7, 0.7, 1, 0.5), m_coralWheels))
        .onFalse(new InstantCommand(() -> m_coralWheels.v_coralOutput(0, 0, 0, 0)));

        //ALGAE ARMS
    m_driverController.a()
        .whileTrue(new InstantCommand(() -> m_algaeArms.v_jog(0.3)))
        .onFalse(new InstantCommand(() -> m_algaeArms.v_jog(0)));
    m_driverController.y()
        .whileTrue(new InstantCommand(() -> m_algaeArms.v_jog(-0.3)))
        .onFalse(new InstantCommand(() -> m_algaeArms.v_jog(0)));

        //CORAL ARM
    m_driverController.x()
        .whileTrue(new InstantCommand(() -> m_coralArm.v_jog(-0.3)))
        .whileFalse(new InstantCommand(() -> m_coralArm.v_jog(0)));
    m_driverController.b()
        .whileTrue(new InstantCommand(() -> m_coralArm.v_jog(0.3)))
        .whileFalse(new InstantCommand(() -> m_coralArm.v_jog(0)));

        //MANUAL ELEVATOR
    m_driverController.povUp()
        .whileTrue(new InstantCommand(() -> m_elevatorSubsystem.v_jog(0.15)))
        .whileFalse(new InstantCommand(() -> m_elevatorSubsystem.v_jog(0)));

    m_driverController.povDown()
        .whileTrue(new InstantCommand(() -> m_elevatorSubsystem.v_jog(-0.15)))
        .whileFalse(new InstantCommand(() -> m_elevatorSubsystem.v_jog(0)));

    //HOME ALL MANIPULATORS
    m_driverController.rightStick().onTrue
        (new HomeAllManipulators(m_elevatorSubsystem, m_coralArm, m_coralRotator, m_algaeArms, m_connectorX));


    // OPERATOR BUTTON BOARD

        // ELEVATOR
    m_operatorBoard.button(1).onTrue
        (new CoralL4(m_elevatorSubsystem, m_coralArm, m_coralRotator, m_connectorX));

    m_operatorBoard.button(4).onTrue
        (new CoralL3(m_elevatorSubsystem, m_coralArm, m_coralRotator, m_connectorX));

    m_operatorBoard.button(7).onTrue
        (new CoralL2(m_elevatorSubsystem, m_coralArm, m_coralRotator, m_connectorX));

    
        // ALGAE CLAW
    m_operatorBoard.button(2).onTrue
        (new Barge(m_elevatorSubsystem, m_algaeArms, m_connectorX));

    m_operatorBoard.button(5).onTrue
        (new AlgaeArmHome(m_algaeArms));

    m_operatorBoard.button(8).onTrue
        (new AlgaeProcessor(m_algaeArms));

    m_operatorBoard.button(11).onTrue
        (new AlgaeIntake(m_elevatorSubsystem, m_algaeArms, m_connectorX));

    
        // CORAL CLAW
    m_operatorBoard.button(3).onTrue
        (new CoralStation(m_elevatorSubsystem, m_coralArm, m_coralRotator, m_connectorX));

    m_operatorBoard.button(6).onTrue
        (new CoralArmHome(m_coralArm, m_coralRotator));

    m_operatorBoard.button(9).onTrue
        (new CoralTrough(m_elevatorSubsystem, m_coralArm, m_coralRotator, m_connectorX));

    m_operatorBoard.button(12).onTrue
        (new CoralIntake(m_elevatorSubsystem, m_coralArm, m_coralRotator, m_connectorX));
  }

  public Command getAutonomousCommand() {

      return autoChooser.getSelected();
  }
}
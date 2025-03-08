// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import java.io.IOException;
import java.util.logging.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;

public class DriveSubsystem extends SubsystemBase {

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  public static final Translation2d[] moduleTranslations = 
    new Translation2d[] {
      new Translation2d(Units.inchesToMeters(14), Units.inchesToMeters(14)),
      new Translation2d(Units.inchesToMeters(14), Units.inchesToMeters(-14)),
      new Translation2d(Units.inchesToMeters(-14), Units.inchesToMeters(14)),
      new Translation2d(Units.inchesToMeters(-14), Units.inchesToMeters(-14))
      };
  
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    Constants.MyConstants.kFrontLeftDrivingCanId,
      Constants.MyConstants.kFrontLeftTurningCanId,
      Constants.MyConstants.kFrontLeftChassisAngularOffset,
      false);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    Constants.MyConstants.kFrontRightDrivingCanId,
      Constants.MyConstants.kFrontRightTurningCanId,
      Constants.MyConstants.kFrontRightChassisAngularOffset,
      false);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
    Constants.MyConstants.kRearLeftDrivingCanId,
      Constants.MyConstants.kRearLeftTurningCanId,
      Constants.MyConstants.kBackLeftChassisAngularOffset,
      false);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
    Constants.MyConstants.kRearRightDrivingCanId,
    Constants.MyConstants.kRearRightTurningCanId,
    Constants.MyConstants.kBackRightChassisAngularOffset,
      false);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(0);

  // Odometry class for tracking robot pose
  // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  //     DriveConstants.kDriveKinematics,
  //     m_gyro.getRotation2d(),
  //     new SwerveModulePosition[] {
  //         m_frontLeft.getPosition(),
  //         m_frontRight.getPosition(),
  //         m_rearLeft.getPosition(),
  //         m_rearRight.getPosition()
  //     },
    SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      Constants.MyConstants.kDriveKinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },

      new Pose2d());

      private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

      

  /** Creates a new DriveSubsystem. 
  * @throws ParseException 
  * @throws IOException */
  

  public DriveSubsystem() throws IOException, ParseException  {

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    RobotConfig config = RobotConfig.fromGUISettings();
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)
        ),
        config,
        () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
    );

    // Configure AutoBuilder last
    // AutoBuilder.configure(
    //         this::getPose, // Robot pose supplier
    //         this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //         ),
    //         Constants.config, // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );

}

    // AutoBuilder.configureHolonomic(
    //   this::getPose,
    //   this::resetPose,
    //   this::getRobotRelativeSpeeds,
    //   this::driveRobotRelative,
    //   new PPHolonomicDriveController(
    //     new PIDConstants(5.0, 0.0, 0.0),
    //     new PIDConstants(5.0, 0.0, 0.0),
    //     new RobotConfig(70, 6, 1.2, new ModuleConfig(
    //       Units.inchesToMeters(1.5), 4.5, 1.2, 60, 50, 1),
    //       moduleTranslations),
    //       () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
    //       this,
    //   () -> {
    //     var alliance = DriverStation.getAlliance();
    //     if (alliance.isPresent()) {
    //       return alliance.get() == DriverStation.Alliance.Red;
    //     }
    //     return false;
    //   },
    //   this
    // );
    
  

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates = 
    Constants.MyConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.MyConstants.kMaxSpeedMetersPerSecond);
  
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.MyConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param b 
   */
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean b) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * Constants.MyConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * Constants.MyConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * Constants.MyConstants.kMaxAngularSpeed;

    

    var swerveModuleStates = Constants.MyConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                m_gyro.getRotation2d())

            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.MyConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.MyConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

 

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getRotation2d().getDegrees(), 360);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.MyConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public Pose2d getPose2d() {
    return m_odometry.getEstimatedPosition();
  }

}
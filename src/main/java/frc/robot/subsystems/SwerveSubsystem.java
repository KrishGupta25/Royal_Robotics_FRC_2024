// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftTurnAbsoluteEncoderPort);

    public final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightTurnAbsoluteEncoderPort);

    public final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftTurningMotorReversed,
            DriveConstants.kBackLeftTurnAbsoluteEncoderPort);

    public final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightTurningMotorReversed,
            DriveConstants.kBackRightTurnAbsoluteEncoderPort);

    public AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
    new Rotation2d(0),  new SwerveModulePosition[] {
      frontRight.getPosition(),
      frontLeft.getPosition(),
      backRight.getPosition(),
      backLeft.getPosition()
    });
    

    public final Field2d m_field = new Field2d();

    public SwerveSubsystem() 
    {
      new Thread(()->{
        try{
          Thread.sleep(1000);
          zeroHeading();
        } catch (Exception e){
        }}).start();
        
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdemetry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveFieldRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
                        AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                        Units.inchesToMeters(15.851360351717), // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }
    
    public void zeroHeading()
    {
      gyro.reset();
    }
    
    public double getHeading()
    {
      return Math.IEEEremainder(-gyro.getAngle(),360);
    }

    public Rotation2d getRotation2d() {
      return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose()
    {
      return odometer.getPoseMeters();
    }
  
    
    public void resetOdemetry(Pose2d pose)
    {
      odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
        frontRight.getPosition(),
        frontLeft.getPosition(),
        backRight.getPosition(),
        backLeft.getPosition()
      } ,pose);
    }

    public void stopModules()
    {
      frontLeft.stop();
      frontRight.stop();
      backLeft.stop();
      backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
      frontRight.setDesiredState(desiredStates[0]);
      frontLeft.setDesiredState(desiredStates[1]);
      backRight.setDesiredState(desiredStates[2]);
      backLeft.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates()
    {
      SwerveModuleState[] states = new SwerveModuleState[] {
        frontRight.getState(),
        frontLeft.getState(),
        backRight.getState(),
        backLeft.getState()
      };

      return states;
    }

    public ChassisSpeeds getSpeeds() 
    {
      return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
      driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
  
      SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
      setModuleStates(targetStates);
    }

          
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("FR Get Position", frontRight.getPosition().toString());
          
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()});

      SmartDashboard.putData("Field", m_field);
      SmartDashboard.putString("Robot Pose", getPose().getTranslation().toString());

      // Do this in either robot periodic or subsystem periodic
      m_field.setRobotPose(odometer.getPoseMeters());

    
  }
}

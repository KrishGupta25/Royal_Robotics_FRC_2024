// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

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

      frontLeft.setDrivePosition(0);
      frontRight.setDrivePosition(0);
      backLeft.setDrivePosition(0);
      backRight.setDrivePosition(0);
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

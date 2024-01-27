// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
  /** Creates a new SwerveJoysticks. */
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;

  
  public SwerveJoystickCmd(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) 
            {
              this.swerveSubsystem = swerveSubsystem;
              this.xSpdFunction = xSpdFunction;
              this.ySpdFunction = ySpdFunction;
              this.turningSpdFunction = turningSpdFunction;
              this.fieldOrientedFunction = fieldOrientedFunction;

              addRequirements(swerveSubsystem);
            }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    swerveSubsystem.frontRight.setBrake();
    swerveSubsystem.frontLeft.setBrake();
    swerveSubsystem.backRight.setBrake();
    swerveSubsystem.backLeft.setBrake();
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
  // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > Constants.OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.OIConstants.kDeadband ? turningSpeed : 0.0;
/*
        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
*/
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);

        SmartDashboard.putNumber("FR Drive Encoder", swerveSubsystem.frontRight.getDrivePosition());
        SmartDashboard.putNumber("FL Drive Encoder", swerveSubsystem.frontLeft.getDrivePosition());
        SmartDashboard.putNumber("BR Drive Encoder", swerveSubsystem.backRight.getDrivePosition());
        SmartDashboard.putNumber("BL Drive Encoder", swerveSubsystem.backLeft.getDrivePosition());

        SmartDashboard.putNumber("FR Turning Encoder", swerveSubsystem.frontRight.getAbsoluteEncoderPositon());
        SmartDashboard.putNumber("FL Turning Encoder", swerveSubsystem.frontLeft.getAbsoluteEncoderPositon());
        SmartDashboard.putNumber("BR Turning Encoder", swerveSubsystem.backRight.getAbsoluteEncoderPositon());
        SmartDashboard.putNumber("BL Turning Encoder", swerveSubsystem.backLeft.getAbsoluteEncoderPositon());

        SmartDashboard.putBoolean("FR Drive Rev", swerveSubsystem.frontRight.getDriveMotorRev());
        SmartDashboard.putBoolean("FL Drive Rev", swerveSubsystem.frontLeft.getDriveMotorRev());
        SmartDashboard.putBoolean("BR Drive Rev", swerveSubsystem.backRight.getDriveMotorRev());
        SmartDashboard.putBoolean("BL Drive Rev", swerveSubsystem.backLeft.getDriveMotorRev());
        
        SmartDashboard.putBoolean("FR Turn Rev", swerveSubsystem.frontRight.getTurnMotorRev());
        SmartDashboard.putBoolean("FL Turn Rev", swerveSubsystem.frontLeft.getTurnMotorRev());
        SmartDashboard.putBoolean("BR Turn Rev", swerveSubsystem.backRight.getTurnMotorRev());
        SmartDashboard.putBoolean("BL Turn Rev", swerveSubsystem.backLeft.getTurnMotorRev());

        
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

  
}


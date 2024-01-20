// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem<xboxButton> extends SubsystemBase {
  
    CANSparkMax leftMotor = new CANSparkMax(14, MotorType.kBrushless);
    CANSparkMax  rightMotor = new CANSparkMax(15, MotorType.kBrushless);

    /** Creates a new IntakeWithA. */
  public IntakeSubsystem() {

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

  }

public void setMotors(double leftSpeed, double rightSpeed) {
  leftMotor.set(leftSpeed);
  rightMotor.set(rightSpeed);
}


  @Override
  public void periodic() {}
}
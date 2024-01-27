package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    
    private final CANSparkMax driveMotor; //Initializing drive motor controller 
    private final CANSparkMax turnMotor; //Initializing turn motor controller 

    private final RelativeEncoder driveBuiltInEncoder; //Initializing drive motor encoder
    //private final RelativeEncoder turnBuiltInEncoder; //Initializing turn motor encoder

    private final PIDController turningPidController; //Initializing PID controller to turn the modules with the joystick
    
    private final CANcoder CANabsoluterEncoder; //Initializing CANCoder to always know where the wheels are rotated
    
    /*
    I know its in the video but we dont use these, the video is outdated. We go into the pheonix tuner and manually do this

        private final boolean absoluteEncoderReversed;
        private final double absoluteEncoderOffset;
    
    */
    public boolean driveMotorRev;
    public boolean turnMotorRev;
    
    public SwerveModule(int driveMotorCANID,
                        int turnMotorCANID,
                        boolean driveMotorRevered,
                        boolean turnMotorReversed,
                        int absoluteEncoderCANID)
    {
        CANabsoluterEncoder = new CANcoder(absoluteEncoderCANID);
        
        //Giving the motor a CANid and making it a brushless motor
        driveMotor = new CANSparkMax(driveMotorCANID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorCANID, MotorType.kBrushless);
        
        //Based off what we feed to the constructor, is the motor reversed? (SwerveSubsystem.java)
        driveMotor.setInverted(driveMotorRevered);
        turnMotor.setInverted(turnMotorReversed); 

        driveMotorRev = driveMotorRevered;
        turnMotorRev = turnMotorReversed;

        //Telling it what mode to be in while not getting user input
        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kCoast);

        //Current limiting the motors
        driveMotor.setSmartCurrentLimit(50); 
        turnMotor.setSmartCurrentLimit(50);

        //Getting the value of the built in encoders of the motors
        driveBuiltInEncoder = driveMotor.getEncoder();

        //Setting the conversion factors for the encoders
        //driveBuiltInEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter);
        //driveBuiltInEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        //Making the pid stuff for turning the wheels per joytick output
        turningPidController = new PIDController(Constants.ModuleConstants.kTurnP,0, 0);

        //Making sure the output is between 180 and -180 degrees
        turningPidController.enableContinuousInput(-180, 180);

        resetEncoders();
    }

    //Getting the position of both motors
    public double getDrivePosition()
    {
        return driveBuiltInEncoder.getPosition();
    }

    public boolean getDriveMotorRev()
    {
        return driveMotorRev;
    }

    public boolean getTurnMotorRev()
    {
        return turnMotorRev;
    }

    public void setVoltage(double volts)
    {
        driveMotor.set(volts);
    }

    public void setCoast()
    {
        driveMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setBrake()
    {
        driveMotor.setIdleMode(IdleMode.kBrake);
    }

    //Getting the velocity of both motors
    public double getDriveVelocity()
    {
        return driveBuiltInEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderPositon()*(Math.PI/180)));
        //return new SwerveModulePosition();
    }
    
    public double getAbsoluteEncoderPositon() //Getting the angle of the absolute encoder in degrees
    {
        return CANabsoluterEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }
    
    public void resetEncoders() //Reseting Encoders
    {
        driveBuiltInEncoder.setPosition(0);
    }
   
    public void setToAngle(double angle)
    {
        turnMotor.set(turningPidController.calculate(getAbsoluteEncoderPositon(), angle));
    }

    public void setDrivePosition(double pos)
    {
        driveBuiltInEncoder.setPosition(pos);
    }

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(getDriveVelocity(),new Rotation2d(getAbsoluteEncoderPositon()*(Math.PI/180)));      
    }

    public void setDesiredState(SwerveModuleState state)
    {
        if(Math.abs(state.speedMetersPerSecond) <= 0.01)
        {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        turnMotor.set(turningPidController.calculate(getAbsoluteEncoderPositon(), state.angle.getDegrees()));
        // .getdegrees? Should the optimization also be that? need to print those values and see the difference between
        // .getdegrees and .getangle
    }
    
    public void stop()
    {
        driveMotor.set(0);
        turnMotor.set(0);
    }


}

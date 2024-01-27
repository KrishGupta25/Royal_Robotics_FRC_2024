package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShootSubsystem extends SubsystemBase {

    private CANSparkMax shootMotor1 = new CANSparkMax(21, MotorType.kBrushless); 
    private CANSparkMax shootMotor2 = new CANSparkMax(1, MotorType.kBrushless);
    
    public ShootSubsystem() {
        shootMotor1.restoreFactoryDefaults();
        shootMotor2.restoreFactoryDefaults();
        
        shootMotor1.setInverted(true);
        shootMotor2.setInverted(false);
    }
    public void setPosition(double speed) {
            shootMotor1.set(speed);
            shootMotor2.set(speed);
    }
     
}

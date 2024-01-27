package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;

public class ShootActiveCmd extends Command  {

    private ShootSubsystem shootSubsystem;

    public ShootActiveCmd(ShootSubsystem shootSubsystem) {
        this.shootSubsystem = shootSubsystem;
        addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ShootActiveCmd started!");
    }

    @Override
    public void execute() {
        shootSubsystem.setPosition(0.5);
    }

    @Override
    public void end(boolean interupted) {
        System.out.println("ShootActiveCmd ended!");
        shootSubsystem.setPosition(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    

    
}

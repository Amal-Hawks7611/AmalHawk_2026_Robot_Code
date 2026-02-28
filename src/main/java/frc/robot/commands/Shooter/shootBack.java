package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EnabledParts;
import frc.robot.subsystems.ShooterSubsystem;

public class shootBack extends Command {
    public final ShooterSubsystem shooterSubsystem;
    public static Timer timer3;
    public static Boolean isShot;
    public shootBack(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        timer3 = new Timer();
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Fuel BackShoot Ininialized");
        shooterSubsystem.setShooting(true);
        timer3.restart();
        isShot = false;
    }

    @Override
    public void execute() {
        if (EnabledParts.IS_SHOOTER_ENABLED && !timer3.hasElapsed(0.3)){
            shooterSubsystem.ShootBack();
        }else{
            this.end(false);
        }
        }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Fuel ShotBack Ended");
        shooterSubsystem.stopMotors();
        shooterSubsystem.setShooting(false);
        timer3.reset();
        timer3.stop();
    }

    @Override
    public boolean isFinished() {
        return !shooterSubsystem.isShooting();
    }
}
